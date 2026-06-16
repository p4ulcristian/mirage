/* diag.h - lightweight always-on diagnostic log.
 *
 * Appends timestamped lines to /tmp/mirage-diag.log (override with $MIRAGE_DIAG_LOG)
 * so intermittent glitches - e.g. the wall "jumping while still" - can be analysed
 * after the fact instead of needing to be caught live. Thread-safe; opens lazily.
 * Set MIRAGE_DIAG=0 to disable.
 */
#ifndef MIRAGE_DIAG_H
#define MIRAGE_DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Append one line: "<ms since start> <tag> <formatted...>". printf-style. */
void diag_logf(const char *tag, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

/* True if logging is enabled (cheap; lets callers skip building expensive args). */
int diag_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* MIRAGE_DIAG_H */
