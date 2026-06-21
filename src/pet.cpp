/* pet.cpp - the reactive pet, now a real glTF skinned model (Tier 1).
 *
 * Loads a rigged .glb (a raccoon; bootstraps on the CC0 Fox until the raccoon
 * asset lands), GPU-skins it, plays one of its animation clips, lights+textures
 * it, and places it in the star dome under control of the same gaze/idle state
 * machine the Tier-0 blob used. Still fully behind the pet.h seam - render.cpp is
 * untouched. The procedural swim override (rotating named limb/tail/spine bones)
 * layers on top once we know the raccoon's bone names; until then it plays the
 * model's own best clip.
 *
 * Pipeline per frame: reset joints to rest -> sample the active clip into the
 * animated joints -> walk the hierarchy to model-space joint matrices -> multiply
 * by inverse-bind -> upload the bone palette -> draw. Standard skeletal animation.
 */
#include "pet.h"
#include "mirage.h"
#include "handle.hpp"
#include "cgltf.h"
#include "stb_image.h"

#include <GLES2/gl2.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <string>
#include <print>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PET_MAXB 64   /* bone-palette uniform slots (Apple/Asahi GLES gives plenty) */

/* GPU skinning: blend up to 4 bone matrices per vertex. Joints arrive as floats
 * (GLES2 has no integer vertex attributes). Normal is the skinned normal, lit by
 * one directional light + ambient - cheap but clearly 3D on the additive optics. */
static const char *PET_VERT =
    "attribute vec3 aPos;\n"
    "attribute vec3 aNrm;\n"
    "attribute vec2 aUV;\n"
    "attribute vec4 aJoint;\n"
    "attribute vec4 aWeight;\n"
    "uniform mat4 uVP;\n"
    "uniform mat4 uModel;\n"
    "uniform mat4 uBones[" "64" "];\n"
    "varying highp vec2 vUV;\n"
    "varying highp vec3 vN;\n"
    "void main() {\n"
    "  mat4 skin = aWeight.x * uBones[int(aJoint.x)]\n"
    "            + aWeight.y * uBones[int(aJoint.y)]\n"
    "            + aWeight.z * uBones[int(aJoint.z)]\n"
    "            + aWeight.w * uBones[int(aJoint.w)];\n"
    "  vec4 sp = skin * vec4(aPos, 1.0);\n"
    "  gl_Position = uVP * uModel * sp;\n"
    "  vN = mat3(uModel) * (mat3(skin) * aNrm);\n"
    "  vUV = aUV;\n"
    "}\n";

static const char *PET_FRAG =
    "precision mediump float;\n"
    "varying highp vec2 vUV;\n"
    "varying highp vec3 vN;\n"
    "uniform sampler2D uTex;\n"
    "uniform float uHasTex;\n"
    "uniform vec3 uColor;\n"
    "uniform vec3 uLight;\n"
    "void main() {\n"
    "  vec3 base = (uHasTex > 0.5) ? texture2D(uTex, vUV).rgb : uColor;\n"
    "  float d = max(dot(normalize(vN), normalize(uLight)), 0.0);\n"
    "  vec3 c = base * (0.35 + 0.75 * d);\n"
    "  gl_FragColor = vec4(c, 1.0);\n"
    "}\n";

namespace {

/* ---- GL + model handles ----------------------------------------------------- */
struct PetGL {
    own::GlProgram prog;
    own::GlBuffer  vbo, ibo;
    own::GlTexture tex;
    int   index_count = 0;
    bool  has_tex = false;
    GLint aPos=-1, aNrm=-1, aUV=-1, aJoint=-1, aWeight=-1;
    GLint uVP=-1, uModel=-1, uBones=-1, uTex=-1, uHasTex=-1, uColor=-1, uLight=-1;
} P;

/* one interleaved skinned vertex: pos(3) nrm(3) uv(2) joint(4) weight(4) = 16 f */
struct Vtx { float pos[3], nrm[3], uv[2], joint[4], weight[4]; };

/* ---- skeleton --------------------------------------------------------------- */
struct Node { vec3 t; quat r; vec3 s; int parent; };
std::vector<Node>  g_rest;        /* rest local TRS per glTF node            */
std::vector<Node>  g_pose;        /* working copy, overwritten by the clip   */
std::vector<int>   g_order;       /* node indices, parents before children   */
std::vector<int>   g_joint_node;  /* joint slot -> node index                */
std::vector<mat4>  g_invbind;     /* inverse-bind per joint                  */
int  g_njoints = 0;
bool g_has_skin = false;
mat4 g_norm = {{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}};  /* size+centre the mesh */

/* ---- animation: ALL clips cached; crossfade between them -------------------- */
struct Chan { int node; int path; int interp; int comps;   /* path: 0=T 1=R 2=S */
              std::vector<float> times; std::vector<float> vals; };
struct Clip { std::string name; float dur; std::vector<Chan> chans; };
std::vector<Clip> g_clips;
int   g_clipA=-1, g_clipB=-1;          /* current + fading-out clip            */
float g_tA=0, g_tB=0, g_blend=1;       /* playheads + A weight (1 = fully A)   */
int   g_walk_clip=-1, g_idle_clip=-1, g_rear_clip=-1;

/* ---- behaviour: a little raccoon brain pacing a window's top edge ----------- */
enum Beh { B_AMBLE, B_PAUSE, B_REAR, B_DASH };
std::vector<int> g_lock;        /* root nodes whose clip translation we cancel  */
float  scene_d = 1.2f;
float  g_model_h = 0.5f;        /* the model's world height (set at load)       */
float  face_yaw = 0;            /* smoothed heading                             */
int    g_screen = -1;           /* which window he's walking on                 */
vec3   pos = {0,0,0};
Beh    beh = B_AMBLE;
float  beh_t = 0, beh_dur = 4;  /* time in current state + its planned length   */
float  u01 = 0.5f;              /* position along the top edge, 0..1            */
float  wdir = 1;                /* walk direction, +1/-1                        */
float  curspeed = 0;            /* eased current speed (edge fraction/sec)      */
float  pace = 1;                /* per-amble speed jitter so laps aren't equal  */
bool   ts_seeded = false;
timespec prev_ts;

/* tunables - all in one place so it's easy to dial in by eye */
const bool  PET_CHILL = true;       /* park him idle in one spot; disables the roam brain  */
const float TARGET_H_FRAC = 3.0f;   /* model height vs screen distance (big chill raccoon) */
const float AMBLE_SPD = 0.09f;      /* edge fraction/sec ambling (~11s across; calm)       */
const float DASH_SPD  = 0.30f;      /* edge fraction/sec scurrying                         */
const float FEET_LIFT = 0.5f;       /* model heights above the edge (0.5 = feet on edge)   */
const float PERCH_OUT = 0.6f;       /* model heights pushed toward viewer so he sits IN    */
                                    /* FRONT of the screen plane, not embedded in it       */
const float BLEND_TIME = 0.28f;     /* clip crossfade seconds                              */
const float FACE_YAW_OFF = 0.0f;    /* if he faces sideways/backwards, nudge +-PI/2 or PI  */

/* transform a point (w=1) by a column-major mat4. */
vec3 xform(const mat4 &M, vec3 p){
    return v3(M.m[0]*p.x + M.m[4]*p.y + M.m[8]*p.z + M.m[12],
              M.m[1]*p.x + M.m[5]*p.y + M.m[9]*p.z + M.m[13],
              M.m[2]*p.x + M.m[6]*p.y + M.m[10]*p.z + M.m[14]);
}

float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }
float ease(float k, float dt){ return 1.0f - expf(-k*dt); }
vec3  v3_lerp(vec3 a, vec3 b, float t){ return v3(a.x+(b.x-a.x)*t, a.y+(b.y-a.y)*t, a.z+(b.z-a.z)*t); }
float frand(){ return (float)rand()/(float)RAND_MAX; }
float rrange(float a, float b){ return a + (b-a)*frand(); }

mat4 trs(const Node &n){
    return m4_mul(m4_translate(n.t), m4_mul(m4_from_quat(n.r), m4_scale(n.s)));
}

/* decompose a column-major mat4 into T/R/S (for the rare matrix-baked node). */
void decompose(const float m[16], vec3 &T, quat &R, vec3 &S){
    T = v3(m[12], m[13], m[14]);
    vec3 c0 = v3(m[0],m[1],m[2]), c1 = v3(m[4],m[5],m[6]), c2 = v3(m[8],m[9],m[10]);
    S = v3(v3_len(c0), v3_len(c1), v3_len(c2));
    if (S.x>1e-8f){c0=v3_scale(c0,1/S.x);} if (S.y>1e-8f){c1=v3_scale(c1,1/S.y);} if (S.z>1e-8f){c2=v3_scale(c2,1/S.z);}
    float tr = c0.x + c1.y + c2.z;
    quat q;
    if (tr > 0){ float s=sqrtf(tr+1)*2; q.w=0.25f*s; q.x=(c1.z-c2.y)/s; q.y=(c2.x-c0.z)/s; q.z=(c0.y-c1.x)/s; }
    else if (c0.x>c1.y && c0.x>c2.z){ float s=sqrtf(1+c0.x-c1.y-c2.z)*2; q.w=(c1.z-c2.y)/s; q.x=0.25f*s; q.y=(c1.x+c0.y)/s; q.z=(c2.x+c0.z)/s; }
    else if (c1.y>c2.z){ float s=sqrtf(1+c1.y-c0.x-c2.z)*2; q.w=(c2.x-c0.z)/s; q.x=(c1.x+c0.y)/s; q.y=0.25f*s; q.z=(c2.y+c1.z)/s; }
    else { float s=sqrtf(1+c2.z-c0.x-c1.y)*2; q.w=(c0.y-c1.x)/s; q.x=(c2.x+c0.z)/s; q.y=(c2.y+c1.z)/s; q.z=0.25f*s; }
    R = q_norm(q);
}

Node node_rest(const cgltf_node *n){
    Node out; out.parent = -1; out.t=v3(0,0,0); out.r=q_identity(); out.s=v3(1,1,1);
    if (n->has_matrix) { decompose(n->matrix, out.t, out.r, out.s); return out; }
    if (n->has_translation) out.t = v3(n->translation[0], n->translation[1], n->translation[2]);
    if (n->has_rotation)    out.r = q_norm((quat){n->rotation[3], n->rotation[0], n->rotation[1], n->rotation[2]});
    if (n->has_scale)       out.s = v3(n->scale[0], n->scale[1], n->scale[2]);
    return out;
}

static GLuint compile(GLenum type, const char *src){
    GLuint s = glCreateShader(type);
    glShaderSource(s,1,&src,nullptr); glCompileShader(s);
    GLint ok=0; glGetShaderiv(s,GL_COMPILE_STATUS,&ok);
    if(!ok){ char log[1024]; glGetShaderInfoLog(s,sizeof log,nullptr,log);
             std::print(stderr,"pet: shader compile failed: {}\n",log); glDeleteShader(s); return 0; }
    return s;
}

/* upload the model's base-colour texture (embedded in glb, or external file). */
bool load_texture(const cgltf_data *data, const cgltf_image *img, const char *model_path){
    if (!img) return false;
    int w=0,h=0,n=0; unsigned char *px=nullptr;
    if (img->buffer_view && img->buffer_view->buffer && img->buffer_view->buffer->data){
        const unsigned char *src = (const unsigned char*)img->buffer_view->buffer->data + img->buffer_view->offset;
        px = stbi_load_from_memory(src, (int)img->buffer_view->size, &w,&h,&n, 4);
    } else if (img->uri && strncmp(img->uri,"data:",5)!=0){
        std::string dir(model_path); auto sl=dir.find_last_of('/');
        dir = (sl==std::string::npos)? std::string("") : dir.substr(0,sl+1);
        px = stbi_load((dir+img->uri).c_str(), &w,&h,&n, 4);
    }
    (void)data;
    if (!px){ std::print(stderr,"pet: texture decode failed\n"); return false; }
    P.tex.gen();
    glBindTexture(GL_TEXTURE_2D, P.tex);
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,w,h,0,GL_RGBA,GL_UNSIGNED_BYTE,px);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
    stbi_image_free(px);
    std::print(stderr,"pet: texture {}x{}\n",w,h);
    return true;
}

/* Build the skinned vertex/index buffers + skeleton from the first skinned
 * primitive. Returns false (pet disabled) on anything unexpected. */
bool load_model(const char *path){
    cgltf_options opt = {};
    cgltf_data *data = nullptr;
    if (cgltf_parse_file(&opt, path, &data) != cgltf_result_success){
        std::print(stderr,"pet: cgltf parse failed {}\n",path); return false; }
    if (cgltf_load_buffers(&opt, data, path) != cgltf_result_success){
        std::print(stderr,"pet: cgltf buffers failed\n"); cgltf_free(data); return false; }

    /* pick the first primitive that has positions (prefer one with a skin). */
    cgltf_mesh *mesh = nullptr; cgltf_primitive *prim = nullptr;
    for (cgltf_size i=0;i<data->meshes_count && !prim;i++)
        for (cgltf_size j=0;j<data->meshes[i].primitives_count;j++){
            cgltf_primitive *pr = &data->meshes[i].primitives[j];
            for (cgltf_size a=0;a<pr->attributes_count;a++)
                if (pr->attributes[a].type==cgltf_attribute_type_position){ mesh=&data->meshes[i]; prim=pr; break; }
            if (prim) break;
        }
    if (!prim){ std::print(stderr,"pet: no mesh primitive\n"); cgltf_free(data); return false; }

    const cgltf_accessor *aPos=nullptr,*aNrm=nullptr,*aUV=nullptr,*aJ=nullptr,*aW=nullptr;
    for (cgltf_size a=0;a<prim->attributes_count;a++){
        cgltf_attribute *at=&prim->attributes[a];
        switch(at->type){
            case cgltf_attribute_type_position: aPos=at->data; break;
            case cgltf_attribute_type_normal:   aNrm=at->data; break;
            case cgltf_attribute_type_texcoord: if(!aUV) aUV=at->data; break;
            case cgltf_attribute_type_joints:   if(!aJ)  aJ =at->data; break;
            case cgltf_attribute_type_weights:  if(!aW)  aW =at->data; break;
            default: break;
        }
    }
    size_t vcount = aPos->count;
    std::vector<Vtx> verts(vcount);
    vec3 bmin=v3(1e9f,1e9f,1e9f), bmax=v3(-1e9f,-1e9f,-1e9f);
    for (size_t i=0;i<vcount;i++){
        Vtx &v=verts[i]; memset(&v,0,sizeof v);
        cgltf_accessor_read_float(aPos,i,v.pos,3);
        bmin=v3(fminf(bmin.x,v.pos[0]),fminf(bmin.y,v.pos[1]),fminf(bmin.z,v.pos[2]));
        bmax=v3(fmaxf(bmax.x,v.pos[0]),fmaxf(bmax.y,v.pos[1]),fmaxf(bmax.z,v.pos[2]));
        if (aNrm) cgltf_accessor_read_float(aNrm,i,v.nrm,3);
        if (aUV)  cgltf_accessor_read_float(aUV,i,v.uv,2);
        v.weight[0]=1;
        if (aJ){ cgltf_uint ji[4]={0,0,0,0}; cgltf_accessor_read_uint(aJ,i,ji,4);
                 for(int k=0;k<4;k++) v.joint[k]=(float)ji[k]; }
        if (aW){ cgltf_accessor_read_float(aW,i,v.weight,4);
                 float s=v.weight[0]+v.weight[1]+v.weight[2]+v.weight[3];
                 if (s>1e-6f){ for(int k=0;k<4;k++) v.weight[k]/=s; } }
    }

    /* indices */
    std::vector<unsigned> idx;
    if (prim->indices){ idx.resize(prim->indices->count);
        for (size_t i=0;i<idx.size();i++) idx[i]=(unsigned)cgltf_accessor_read_index(prim->indices,i); }
    else { idx.resize(vcount); for(size_t i=0;i<vcount;i++) idx[i]=(unsigned)i; }

    /* compute smooth normals if the model didn't ship them (e.g. the Fox) */
    if (!aNrm){
        for (auto &v:verts){ v.nrm[0]=v.nrm[1]=v.nrm[2]=0; }
        for (size_t i=0;i+2<idx.size();i+=3){
            Vtx &a=verts[idx[i]],&b=verts[idx[i+1]],&c=verts[idx[i+2]];
            vec3 e1=v3(b.pos[0]-a.pos[0],b.pos[1]-a.pos[1],b.pos[2]-a.pos[2]);
            vec3 e2=v3(c.pos[0]-a.pos[0],c.pos[1]-a.pos[1],c.pos[2]-a.pos[2]);
            vec3 nf=v3_cross(e1,e2);
            for (int k=0;k<3;k++){ Vtx &vv=verts[idx[i+k]]; vv.nrm[0]+=nf.x; vv.nrm[1]+=nf.y; vv.nrm[2]+=nf.z; }
        }
        for (auto &v:verts){ vec3 nn=v3_norm(v3(v.nrm[0],v.nrm[1],v.nrm[2])); v.nrm[0]=nn.x;v.nrm[1]=nn.y;v.nrm[2]=nn.z; }
    }

    /* normalize: uniform scale to a target height, recentre on the AABB middle */
    vec3 c = v3_scale(v3_add(bmin,bmax),0.5f);
    float hgt = fmaxf(bmax.y-bmin.y, 1e-4f);
    float target_h = TARGET_H_FRAC * scene_d;  /* world height of the critter (m) */
    g_model_h = target_h;
    float sc = target_h / hgt;
    g_norm = m4_mul(m4_scale(v3(sc,sc,sc)), m4_translate(v3_scale(c,-1.0f)));

    /* skeleton */
    g_has_skin = (mesh && prim && data->skins_count>0 && aJ && aW);
    if (g_has_skin){
        cgltf_skin *skin = &data->skins[0];
        g_njoints = (int)skin->joints_count;
        if (g_njoints > PET_MAXB){ std::print(stderr,"pet: {} joints > {} - skinning off\n",g_njoints,PET_MAXB); g_has_skin=false; }
    }
    if (g_has_skin){
        cgltf_skin *skin = &data->skins[0];
        size_t nn = data->nodes_count;
        g_rest.resize(nn);
        for (size_t i=0;i<nn;i++){ g_rest[i]=node_rest(&data->nodes[i]); }
        for (size_t i=0;i<nn;i++){
            cgltf_node *node=&data->nodes[i];
            for (cgltf_size ch=0; ch<node->children_count; ch++){
                size_t ci=(size_t)(node->children[ch]-data->nodes); if (ci<nn) g_rest[ci].parent=(int)i;
            }
        }
        /* topological order: roots first (so global = global[parent]*local works) */
        g_order.clear(); g_order.reserve(nn);
        std::vector<char> done(nn,0);
        bool progress=true;
        while (g_order.size()<nn && progress){ progress=false;
            for (size_t i=0;i<nn;i++){ if(done[i])continue;
                int p=g_rest[i].parent;
                if (p<0 || done[p]){ g_order.push_back((int)i); done[i]=1; progress=true; } } }
        for (size_t i=0;i<nn;i++) if(!done[i]) g_order.push_back((int)i);   /* cycle guard */

        /* find the locomotion root(s): the clip translates these to "walk forward";
         * we cancel that so the cycle plays in place and our orbit does the moving. */
        g_lock.clear();
        for (size_t i=0;i<nn;i++){ const char *nm=data->nodes[i].name;
            if (nm && (strstr(nm,"rootJoint") || strstr(nm,"Root_M") || strstr(nm,"Hips"))) g_lock.push_back((int)i); }

        g_joint_node.resize(g_njoints); g_invbind.resize(g_njoints);
        for (int j=0;j<g_njoints;j++){
            g_joint_node[j]=(int)(skin->joints[j]-data->nodes);
            float m16[16];
            if (skin->inverse_bind_matrices) cgltf_accessor_read_float(skin->inverse_bind_matrices,j,m16,16);
            else { mat4 id=m4_identity(); memcpy(m16,id.m,sizeof m16); }
            memcpy(g_invbind[j].m,m16,sizeof m16);
        }
        g_pose = g_rest;

        /* cache EVERY clip; the behaviour switches between them for variety. */
        for (cgltf_size ai=0; ai<data->animations_count; ai++){
            cgltf_animation *an=&data->animations[ai];
            Clip clip; clip.name = an->name?an->name:""; clip.dur=0;
            for (cgltf_size ci=0; ci<an->channels_count; ci++){
                cgltf_animation_channel *cc=&an->channels[ci];
                if (!cc->target_node || !cc->sampler) continue;
                int path = cc->target_path==cgltf_animation_path_type_translation?0:
                           cc->target_path==cgltf_animation_path_type_rotation?1:
                           cc->target_path==cgltf_animation_path_type_scale?2:-1;
                if (path<0) continue;
                Chan c2; c2.node=(int)(cc->target_node-data->nodes); c2.path=path;
                c2.interp = cc->sampler->interpolation==cgltf_interpolation_type_step?1:
                            cc->sampler->interpolation==cgltf_interpolation_type_cubic_spline?2:0;
                c2.comps = path==1?4:3;
                cgltf_accessor *in=cc->sampler->input,*out=cc->sampler->output;
                c2.times.resize(in->count);
                for (size_t k=0;k<in->count;k++){ float t; cgltf_accessor_read_float(in,k,&t,1); c2.times[k]=t;
                    if (t>clip.dur) clip.dur=t; }
                size_t per = c2.interp==2?3*c2.comps:c2.comps;   /* cubic: in/val/out */
                c2.vals.resize(out->count*per);
                for (size_t k=0;k<out->count;k++) cgltf_accessor_read_float(out,k,&c2.vals[k*per],(int)per);
                clip.chans.push_back(std::move(c2));
            }
            std::print(stderr,"pet: clip[{}] '{}' chans {} dur {:.2f}s\n",(int)ai,clip.name,(int)clip.chans.size(),clip.dur);
            g_clips.push_back(std::move(clip));
        }
        /* resolve the clips the brain uses (case-insensitive name match) */
        auto find1=[&](const char* sub)->int{ for(size_t i=0;i<g_clips.size();i++){
            std::string s=g_clips[i].name; for(auto&ch:s)ch=(char)tolower(ch);
            if(s.find(sub)!=std::string::npos) return (int)i; } return -1; };
        g_walk_clip=find1("quadruped"); if(g_walk_clip<0)g_walk_clip=find1("walk");
        g_idle_clip=find1("_idle");     if(g_idle_clip<0)g_idle_clip=find1("idle");
        g_rear_clip=find1("standup-idle"); if(g_rear_clip<0)g_rear_clip=find1("stand");
        if(g_walk_clip<0) g_walk_clip=g_clips.empty()?-1:0;
        if(g_idle_clip<0) g_idle_clip=g_walk_clip;
        if(g_rear_clip<0) g_rear_clip=g_idle_clip;
        g_clipA=g_walk_clip; g_clipB=-1; g_blend=1; g_tA=0;
        std::print(stderr,"pet: clips walk={} idle={} rear={}\n",g_walk_clip,g_idle_clip,g_rear_clip);
    }

    /* GL upload */
    P.vbo.gen(); glBindBuffer(GL_ARRAY_BUFFER,P.vbo);
    glBufferData(GL_ARRAY_BUFFER,(GLsizeiptr)(verts.size()*sizeof(Vtx)),verts.data(),GL_STATIC_DRAW);
    P.ibo.gen(); glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,P.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,(GLsizeiptr)(idx.size()*sizeof(unsigned)),idx.data(),GL_STATIC_DRAW);
    P.index_count=(int)idx.size();

    /* base-colour texture: metallic-roughness base, or spec-gloss diffuse (this
     * raccoon uses spec-gloss, where the albedo lives in diffuse_texture). */
    if (prim->material){
        cgltf_texture *t=nullptr;
        if (prim->material->has_pbr_metallic_roughness)
            t = prim->material->pbr_metallic_roughness.base_color_texture.texture;
        if (!t && prim->material->has_pbr_specular_glossiness)
            t = prim->material->pbr_specular_glossiness.diffuse_texture.texture;
        if (t && t->image) P.has_tex = load_texture(data,t->image,path);
    }

    std::print(stderr,"pet: model '{}' verts {} tris {} joints {} skin {}\n",
               path, (int)vcount, P.index_count/3, g_njoints, g_has_skin?"yes":"no");
    cgltf_free(data);
    return true;
}

/* sample a channel's value at time t into out[comps]. */
void sample(const Chan &c, float t, float *out){
    size_t n=c.times.size(); if(!n){ for(int i=0;i<c.comps;i++)out[i]=0; return; }
    size_t per = c.interp==2?3*(size_t)c.comps:(size_t)c.comps;
    size_t voff = c.interp==2? (size_t)c.comps : 0;   /* cubic: the value term */
    if (t<=c.times[0]){ for(int i=0;i<c.comps;i++) out[i]=c.vals[0*per+voff+i]; return; }
    if (t>=c.times[n-1]){ for(int i=0;i<c.comps;i++) out[i]=c.vals[(n-1)*per+voff+i]; return; }
    size_t k=0; while (k+1<n && c.times[k+1]<t) k++;
    float t0=c.times[k], t1=c.times[k+1];
    float u = (t1>t0)? (t-t0)/(t1-t0) : 0.0f;
    const float *a=&c.vals[k*per+voff], *b=&c.vals[(k+1)*per+voff];
    if (c.interp==1){ for(int i=0;i<c.comps;i++) out[i]=a[i]; return; }   /* STEP */
    if (c.comps==4){   /* rotation: nlerp shortest-path (xyzw order) */
        quat qa={a[3],a[0],a[1],a[2]}, qb={b[3],b[0],b[1],b[2]};
        quat r=q_nlerp(qa,qb,u); out[0]=r.x; out[1]=r.y; out[2]=r.z; out[3]=r.w;
    } else for(int i=0;i<c.comps;i++) out[i]=a[i]+(b[i]-a[i])*u;
}

/* write a clip's pose (sampled at time t) onto `out` (which the caller seeds to
 * rest). Root translation is cancelled so locomotion clips play in place. */
void apply_clip(int ci, float t, std::vector<Node> &out){
    if (ci<0 || ci>=(int)g_clips.size()) return;
    for (const Chan &c : g_clips[ci].chans){
        float v[4]={0,0,0,0}; sample(c,t,v); Node &nd=out[c.node];
        if (c.path==0) nd.t=v3(v[0],v[1],v[2]);
        else if (c.path==1) nd.r=q_norm((quat){v[3],v[0],v[1],v[2]});
        else nd.s=v3(v[0],v[1],v[2]);
    }
    for (int ln : g_lock) out[ln].t = g_rest[ln].t;
}

/* start crossfading to clip `ci`. */
void play(int ci){
    if (ci<0 || ci==g_clipA) return;
    g_clipB=g_clipA; g_tB=g_tA; g_clipA=ci; g_tA=0; g_blend=0;
}

/* enter a behaviour state: choose its duration and the clip that sells it. */
void enter(Beh b){
    beh=b; beh_t=0;
    switch(b){
    case B_AMBLE: beh_dur=rrange(5.0f,10.0f); pace=rrange(0.8f,1.15f); play(g_walk_clip); break;
    case B_PAUSE: beh_dur=rrange(1.5f,3.5f);  play(g_idle_clip); break;
    case B_REAR:  beh_dur=rrange(2.5f,4.5f);  play(g_rear_clip); break;
    case B_DASH:  beh_dur=rrange(0.8f,1.6f);  play(g_walk_clip); break;
    }
}

} // namespace

void pet_init(struct mirage *m){
    GLuint vs=compile(GL_VERTEX_SHADER,PET_VERT), fs=compile(GL_FRAGMENT_SHADER,PET_FRAG);
    if(!vs||!fs){ std::print(stderr,"pet: disabled (shader)\n"); return; }
    P.prog.create();
    glAttachShader(P.prog,vs); glAttachShader(P.prog,fs);
    glLinkProgram(P.prog);
    GLint ok=0; glGetProgramiv(P.prog,GL_LINK_STATUS,&ok);
    glDeleteShader(vs); glDeleteShader(fs);
    if(!ok){ char log[1024]; glGetProgramInfoLog(P.prog,sizeof log,nullptr,log);
             std::print(stderr,"pet: link failed: {}\n",log); P.prog.reset(); return; }
    P.aPos=glGetAttribLocation(P.prog,"aPos");   P.aNrm=glGetAttribLocation(P.prog,"aNrm");
    P.aUV=glGetAttribLocation(P.prog,"aUV");      P.aJoint=glGetAttribLocation(P.prog,"aJoint");
    P.aWeight=glGetAttribLocation(P.prog,"aWeight");
    P.uVP=glGetUniformLocation(P.prog,"uVP");     P.uModel=glGetUniformLocation(P.prog,"uModel");
    P.uBones=glGetUniformLocation(P.prog,"uBones");P.uTex=glGetUniformLocation(P.prog,"uTex");
    P.uHasTex=glGetUniformLocation(P.prog,"uHasTex");P.uColor=glGetUniformLocation(P.prog,"uColor");
    P.uLight=glGetUniformLocation(P.prog,"uLight");

    scene_d = m->cfg.screen_distance_m > 0.1f ? m->cfg.screen_distance_m : 1.2f;

    /* prefer a real raccoon if it's been dropped in; else the dev stand-in */
    const char *paths[]={"assets/raccoon.glb","assets/raccoon/scene.gltf","assets/raccoon-dev.glb"};
    bool loaded=false;
    for (const char *p:paths){ FILE *f=fopen(p,"rb"); if(f){ fclose(f); if(load_model(p)){ loaded=true; break; } } }
    if (!loaded){ std::print(stderr,"pet: no model loaded; disabled\n"); P.prog.reset(); return; }

    face_yaw=0; g_screen=-1; pos=v3(0,0,0);
    u01=0.5f; wdir=1; curspeed=0;
    ts_seeded=false;
    srand((unsigned)time(nullptr));
    enter(B_AMBLE);
    if (PET_CHILL){ u01=0.5f; curspeed=0; enter(B_PAUSE); }   /* sit centered, idle clip */
    std::print(stderr,"pet: ready\n");
}

void pet_draw(struct mirage *m, mat4 vp, vec3 eye_world, quat head){
    (void)m;
    if (!P.prog || P.index_count==0) return;

    timespec now; clock_gettime(CLOCK_MONOTONIC,&now);
    float dt=0; if(ts_seeded) dt=(float)(now.tv_sec-prev_ts.tv_sec)+(float)(now.tv_nsec-prev_ts.tv_nsec)/1e9f;
    prev_ts=now; ts_seeded=true; dt=clampf(dt,0.0f,0.1f);

    /* which window is he on? gaze-nearest, sticky (hysteresis). */
    vec3 view_fwd = q_rotate(head, v3(0,0,-1));
    float d = m->cfg.screen_distance_m > 0.1f ? m->cfg.screen_distance_m : scene_d;
    int n = m->n_screen > 0 ? m->n_screen : m->cfg.screen_count;
    if (n > MIRAGE_MAX_SCREENS) n = MIRAGE_MAX_SCREENS;
    int best=-1; float bestd=-2, curd=-2;
    for (int i=0;i<n;i++){
        if (!m->screen[i].mesh_vbo) continue;
        vec3 cw = xform(layout_model_matrix(m,i), v3(0,0,-d));
        float a = v3_dot(view_fwd, v3_norm(v3_sub(cw, eye_world)));
        if (a>bestd){ bestd=a; best=i; }
        if (i==g_screen) curd=a;
    }
    if (g_screen<0) g_screen=best;
    else if (!PET_CHILL && best>=0 && bestd > curd + 0.06f) g_screen=best;  /* chill: freeze on the first screen */

    /* is he being watched right now? */
    bool looking = v3_dot(view_fwd, v3_norm(v3_sub(pos,eye_world))) > cosf(20.0f*(float)M_PI/180.0f);

    /* ---- the little brain: amble / pause / rear-up to look around / dash ---- */
    beh_t += dt;
    g_blend = clampf(g_blend + dt/BLEND_TIME, 0.0f, 1.0f);
    if (!PET_CHILL) {        /* chill: skip the whole roam brain - he just sits idle */
        (void)looking;
    if (beh_t > beh_dur){
        float r=frand();
        if (looking)           enter(r<0.45f ? B_REAR : B_PAUSE);       /* caught: stop & look */
        else if (beh==B_AMBLE) enter(r<0.5f?B_PAUSE : r<0.85f?B_AMBLE : r<0.95f?B_REAR : B_DASH);
        else                   enter(r<0.85f ? B_AMBLE : B_DASH);       /* after a rest, amble on */
    }
    /* curious: catch him moving and once in a while he'll rear up to check you out
     * (was ~1/sec - far too twitchy; now ~1 per 5s of being watched while moving) */
    if (looking && (beh==B_AMBLE||beh==B_DASH) && beh_t>1.0f && frand()<dt*0.2f) enter(B_REAR);

    /* eased speed (weighty starts/stops) + position along the edge */
    float tgt = (beh==B_DASH)?DASH_SPD : (beh==B_AMBLE)?AMBLE_SPD*pace : 0.0f;
    curspeed += (tgt-curspeed)*ease(4.0f,dt);
    u01 += wdir*curspeed*dt;
    if (u01>1.0f){ u01=1.0f; wdir=-1.0f; if(frand()<0.6f) enter(B_PAUSE); }   /* turn at the corner */
    if (u01<0.0f){ u01=0.0f; wdir=+1.0f; if(frand()<0.6f) enter(B_PAUSE); }
    if (beh==B_AMBLE && frand()<dt*0.04f) wdir=-wdir;                          /* occasional doubling back */
    }

    /* place on the chosen window's top edge; face travel when moving, else you */
    if (g_screen>=0 && g_screen<n && m->screen[g_screen].mesh_vbo){
        screen_t *s=&m->screen[g_screen];
        float arc = s->arc_deg * (float)M_PI/180.0f;
        float aspect = (s->width>0 && s->height>0) ? (float)s->height/(float)s->width : 9.0f/16.0f;
        float hw = d*tanf(arc*0.5f);
        /* top-edge height, matching the mesh: flat = d*tan(arc/2)*aspect, curved
         * strip = d*arc*aspect/2 - so he glues to the real edge, not a flat-chord
         * estimate that floats him off a curved screen (same fix as the #N labels). */
        float hh = (m->cfg.geometry==GEOM_FLAT) ? hw*aspect : d*arc*aspect*0.5f;
        mat4 M = layout_model_matrix(m, g_screen);
        vec3 TL = xform(M, v3(-hw, hh, -d));
        vec3 TR = xform(M, v3( hw, hh, -d));
        vec3 edge = v3_lerp(TL, TR, u01);
        /* push him off the screen plane toward the viewer (local +z points from the
         * screen back to the eye) so his body sits in front of the panel, not through it */
        vec3 fwd = v3_norm(v3_sub(xform(M, v3(0,0,-d+1.0f)), xform(M, v3(0,0,-d))));
        pos = v3(edge.x, edge.y + FEET_LIFT*g_model_h, edge.z);
        pos = v3_add(pos, v3_scale(fwd, PERCH_OUT*g_model_h));

        bool moving = (beh==B_AMBLE||beh==B_DASH) && curspeed>0.02f;
        float yaw;
        if (moving){ vec3 stp=v3_scale(v3_norm(v3_sub(TR,TL)), wdir); yaw=atan2f(stp.x,stp.z); }
        else       { vec3 tv=v3_sub(eye_world,pos);                   yaw=atan2f(tv.x,tv.z); }
        yaw += FACE_YAW_OFF;
        float dy=yaw-face_yaw; while(dy>(float)M_PI)dy-=2*(float)M_PI; while(dy<-(float)M_PI)dy+=2*(float)M_PI;
        face_yaw += dy * ease(6.0f, dt);
    } else {
        pos = v3(0.0f, 0.0f, -0.8f*scene_d);
    }

    /* ---- pose: crossfade clipA over clipB, walk hierarchy, build palette ----- */
    static std::vector<mat4> bones; bones.assign(g_njoints>0?g_njoints:1, m4_identity());
    if (g_has_skin){
        /* advance playheads; the walk cycle's rate tracks movement so feet sync */
        float rate = (g_clipA==g_walk_clip) ? clampf(curspeed/AMBLE_SPD, 0.25f, 4.0f) : 1.0f;
        if (g_clipA>=0){ float dur=g_clips[g_clipA].dur; g_tA+=dt*rate; if(dur>0) g_tA=fmodf(g_tA,dur); }
        if (g_clipB>=0){ float dur=g_clips[g_clipB].dur; g_tB+=dt;      if(dur>0) g_tB=fmodf(g_tB,dur); }

        static std::vector<Node> pA, pB;
        pA = g_rest; apply_clip(g_clipA, g_tA, pA);
        if (g_blend < 0.999f && g_clipB>=0){
            pB = g_rest; apply_clip(g_clipB, g_tB, pB);
            for (size_t i=0;i<g_pose.size();i++){
                g_pose[i].parent = g_rest[i].parent;
                g_pose[i].t = v3_lerp(pB[i].t, pA[i].t, g_blend);
                g_pose[i].r = q_nlerp(pB[i].r, pA[i].r, g_blend);
                g_pose[i].s = v3_lerp(pB[i].s, pA[i].s, g_blend);
            }
        } else { g_pose = pA; }

        std::vector<mat4> global(g_pose.size());
        for (int nn : g_order){ mat4 l=trs(g_pose[nn]); int p=g_pose[nn].parent;
            global[nn] = (p<0)? l : m4_mul(global[p], l); }
        for (int j=0;j<g_njoints;j++) bones[j]=m4_mul(global[g_joint_node[j]], g_invbind[j]);
    }

    mat4 model = m4_mul(m4_translate(pos),
                        m4_mul(m4_from_quat(q_from_euler_ypr(face_yaw,0,0)), g_norm));

    /* ---- draw --------------------------------------------------------------- */
    glUseProgram(P.prog);
    glEnable(GL_DEPTH_TEST); glDepthMask(GL_TRUE); glDisable(GL_BLEND);
    glUniformMatrix4fv(P.uVP,1,GL_FALSE,vp.m);
    glUniformMatrix4fv(P.uModel,1,GL_FALSE,model.m);
    if (g_has_skin && g_njoints>0) glUniformMatrix4fv(P.uBones,g_njoints,GL_FALSE,bones[0].m);
    else { mat4 id=m4_identity(); glUniformMatrix4fv(P.uBones,1,GL_FALSE,id.m); }
    vec3 L=v3_norm(v3(0.4f,1.0f,0.6f));
    glUniform3f(P.uLight,L.x,L.y,L.z);
    glUniform3f(P.uColor,0.55f,0.5f,0.48f);

    glActiveTexture(GL_TEXTURE0);
    if (P.has_tex){ glBindTexture(GL_TEXTURE_2D,P.tex); glUniform1i(P.uTex,0); glUniform1f(P.uHasTex,1.0f); }
    else glUniform1f(P.uHasTex,0.0f);

    glBindBuffer(GL_ARRAY_BUFFER,P.vbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,P.ibo);
    #define ATTR(loc,n,off) do{ glEnableVertexAttribArray(loc); \
        glVertexAttribPointer(loc,n,GL_FLOAT,GL_FALSE,sizeof(Vtx),(void*)(off)); }while(0)
    ATTR(P.aPos,3,offsetof(Vtx,pos));
    ATTR(P.aNrm,3,offsetof(Vtx,nrm));
    ATTR(P.aUV,2,offsetof(Vtx,uv));
    ATTR(P.aJoint,4,offsetof(Vtx,joint));
    ATTR(P.aWeight,4,offsetof(Vtx,weight));
    #undef ATTR
    glDrawElements(GL_TRIANGLES,P.index_count,GL_UNSIGNED_INT,0);

    glDisableVertexAttribArray(P.aPos);  glDisableVertexAttribArray(P.aNrm);
    glDisableVertexAttribArray(P.aUV);   glDisableVertexAttribArray(P.aJoint);
    glDisableVertexAttribArray(P.aWeight);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
}

void pet_finish(void){
    P.prog.reset(); P.vbo.reset(); P.ibo.reset(); P.tex.reset();
    P.index_count=0;
    g_rest.clear(); g_pose.clear(); g_order.clear(); g_joint_node.clear();
    g_invbind.clear(); g_clips.clear(); g_lock.clear();
    g_njoints=0; g_has_skin=false;
    g_clipA=g_clipB=-1; g_blend=1;
}
