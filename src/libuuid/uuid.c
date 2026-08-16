/*
 * 最小 libuuid 实现：不依赖系统 uuid-dev。
 * 随机数来自 /dev/urandom，并按 RFC 4122 标成 UUID version 4。
 */
#include "uuid/uuid.h"

#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

void uuid_generate(uuid_t out)
{
    memset(out, 0, 16);
    int fd = open("/dev/urandom", O_RDONLY);
    if (fd >= 0)
    {
        ssize_t n = read(fd, out, 16);
        (void)n;
        close(fd);
    }
    /* version 4（随机）与 RFC 变体位 */
    out[6] = (unsigned char)((out[6] & 0x0f) | 0x40);
    out[8] = (unsigned char)((out[8] & 0x3f) | 0x80);
}

static void uuid_unparse_impl(const uuid_t uu, char *out, int upper)
{
    const char *fmt = upper
                          ? "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X"
                          : "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x";
    sprintf(out, fmt,
            uu[0], uu[1], uu[2], uu[3], uu[4], uu[5], uu[6], uu[7],
            uu[8], uu[9], uu[10], uu[11], uu[12], uu[13], uu[14], uu[15]);
}

void uuid_generate_random(uuid_t out)
{
    uuid_generate(out);
}

void uuid_generate_time(uuid_t out)
{
    /* 本实现不区分时间戳 UUID，与 random 相同，够 bond/nodelet 使用。 */
    uuid_generate(out);
}

void uuid_unparse(const uuid_t uu, char *out)
{
    uuid_unparse_impl(uu, out, 0);
}

void uuid_unparse_lower(const uuid_t uu, char *out)
{
    uuid_unparse_impl(uu, out, 0);
}

void uuid_unparse_upper(const uuid_t uu, char *out)
{
    uuid_unparse_impl(uu, out, 1);
}

void uuid_copy(uuid_t dst, const uuid_t src)
{
    memcpy(dst, src, 16);
}

void uuid_clear(uuid_t uu)
{
    memset(uu, 0, 16);
}

int uuid_is_null(const uuid_t uu)
{
    int i;
    for (i = 0; i < 16; ++i)
    {
        if (uu[i] != 0)
        {
            return 0;
        }
    }
    return 1;
}

int uuid_compare(const uuid_t uu1, const uuid_t uu2)
{
    return memcmp(uu1, uu2, 16);
}

int uuid_parse(const char *in, uuid_t uu)
{
    int i = 0;
    int hi = 1;
    unsigned char byte = 0;
    const char *p;
    if (in == NULL)
    {
        return -1;
    }
    memset(uu, 0, 16);
    for (p = in; *p && i < 16; ++p)
    {
        if (*p == '-')
        {
            continue;
        }
        unsigned char c = (unsigned char)*p;
        unsigned char nibble;
        if (c >= '0' && c <= '9')
        {
            nibble = (unsigned char)(c - '0');
        }
        else if (c >= 'a' && c <= 'f')
        {
            nibble = (unsigned char)(c - 'a' + 10);
        }
        else if (c >= 'A' && c <= 'F')
        {
            nibble = (unsigned char)(c - 'A' + 10);
        }
        else
        {
            return -1;
        }
        if (hi)
        {
            byte = (unsigned char)(nibble << 4);
            hi = 0;
        }
        else
        {
            uu[i++] = (unsigned char)(byte | nibble);
            hi = 1;
        }
    }
    return i == 16 ? 0 : -1;
}
