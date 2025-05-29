#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define UNITY_FIXTURE_NO_EXTRAS

extern void elog_raw_output(const char *format, ...);
#define UNITY_OUTPUT_CHAR(a) elog_raw_output("%c", a)

#ifdef __cplusplus
}
#endif

#endif /* UNITY_CONFIG_H */
