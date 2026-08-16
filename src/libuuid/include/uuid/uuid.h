#ifndef UUID_UUID_H
#define UUID_UUID_H

/*
 * 本仓库自己实现的 libuuid 兼容头文件（不是从 util-linux 下载）。
 * bondcpp / nodelet 只需要 generate + unparse 等一小部分 API。
 * 用 extern "C" 保证 C++ 链接时符号不被修饰。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned char uuid_t[16];

void uuid_generate(uuid_t out);
void uuid_generate_random(uuid_t out);
void uuid_generate_time(uuid_t out);
void uuid_unparse(const uuid_t uu, char *out);
void uuid_unparse_lower(const uuid_t uu, char *out);
void uuid_unparse_upper(const uuid_t uu, char *out);
void uuid_copy(uuid_t dst, const uuid_t src);
void uuid_clear(uuid_t uu);
int uuid_is_null(const uuid_t uu);
int uuid_compare(const uuid_t uu1, const uuid_t uu2);
int uuid_parse(const char *in, uuid_t uu);

#ifdef __cplusplus
}
#endif

#endif
