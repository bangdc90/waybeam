#ifndef CHILD_SUPERVISOR_H
#define CHILD_SUPERVISOR_H

#include <stddef.h>

#define CHILD_SUPERVISOR_CONFIG_PATH "/etc/waybeam-children.json"

typedef struct ChildSupervisor ChildSupervisor;

int child_supervisor_validate_json(const char *json, size_t length);
int child_supervisor_start(ChildSupervisor **out, const char *config_path,
	const char *vfrm_uri);
void child_supervisor_stop(ChildSupervisor **supervisor);

#endif