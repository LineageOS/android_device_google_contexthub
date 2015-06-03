#ifndef _LIST_H_
#define _LIST_H_

#ifdef __cplusplus
 extern "C" {
#endif

struct link_t
{
    struct link_t *prev, *next;
} typedef link_t;

#define outer_struct(addr, struct_name, link_name) \
    ((struct_name *)((addr_t)(addr) - offsetof(type, member)))

#define list_iterate(list, cur_link, tmp_link) \
    for (cur_link = (list)->next, tmp = (cur_link)->next; \
            cur_link != (list); cur_link = tmp, tmp = (cur_link)->next)

static inline void list_add_tail(struct link_t *list, struct link_t *item)
{
    item->prev = list->prev;
    item->next = list; 
    list->prev->next = item;
    list->prev = item;
}

#ifdef __cplusplus
}
#endif

#endif

