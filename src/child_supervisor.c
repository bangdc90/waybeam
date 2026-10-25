#include "child_supervisor.h"

#include "venc_frame_ring.h"
#include "cJSON.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <spawn.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define CHILD_LIMIT 8
#define CHILD_ARG_LIMIT 64
#define CHILD_ENV_LIMIT 8
#define CHILD_TEXT_LIMIT 256
#define CHILD_ENV_TEXT_LIMIT 512
#define CHILD_JSON_LIMIT (32 * 1024)
#define CHILD_FD_LIMIT 256
#define CHILD_POLL_MS 100

typedef struct {
	char name[CHILD_TEXT_LIMIT];
	char executable[CHILD_TEXT_LIMIT];
	char args[CHILD_ARG_LIMIT][CHILD_TEXT_LIMIT];
	char env_names[CHILD_ENV_LIMIT][32];
	size_t argc;
	size_t env_count;
	int restart_on_failure;
	int restart_exhausted;
	unsigned int backoff_ms;
	unsigned int backoff_max_ms;
	unsigned int restart_limit;
	unsigned int restart_window_sec;
	unsigned int stop_timeout_ms;
	pid_t pid;
	unsigned int restarts;
	struct timespec window_start;
	struct timespec restart_at;
} ChildSpec;

struct ChildSupervisor {
	pthread_mutex_t mutex;
	pthread_cond_t condition;
	pthread_t thread;
	int thread_started;
	int stopping;
	size_t child_count;
	ChildSpec children[CHILD_LIMIT];
	char vfrm_uri[CHILD_TEXT_LIMIT];
};

static int json_string(cJSON *object, const char *key, char *out, size_t size)
{
	cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
	size_t length;
	if (!cJSON_IsString(item) || !item->valuestring)
		return -1;
	length = strlen(item->valuestring);
	if (length == 0 || length >= size)
		return -1;
	memcpy(out, item->valuestring, length + 1);
	return 0;
}

static int json_uint(cJSON *object, const char *key, unsigned int *out,
	unsigned int min, unsigned int max)
{
	cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
	if (!cJSON_IsNumber(item) || item->valuedouble < min ||
	    item->valuedouble > max || item->valuedouble != (double)item->valueint)
		return -1;
	*out = (unsigned int)item->valueint;
	return 0;
}

static int parse_child(cJSON *item, ChildSpec *child)
{
	cJSON *value;
	cJSON *args;
	cJSON *env;
	cJSON *entry;
	char start_after[32];
	char restart[32];
	char stop_signal[32];
	int enabled;

	memset(child, 0, sizeof(*child));
	if (!cJSON_IsObject(item) ||
	    json_string(item, "name", child->name, sizeof(child->name)) != 0 ||
	    json_string(item, "exec", child->executable,
		    sizeof(child->executable)) != 0 || child->executable[0] != '/' ||
	    json_string(item, "startAfter", start_after, sizeof(start_after)) != 0 ||
	    strcmp(start_after, "vfrm-ready") != 0 ||
	    json_string(item, "restart", restart, sizeof(restart)) != 0 ||
	    (strcmp(restart, "on-failure") != 0 && strcmp(restart, "never") != 0) ||
	    json_string(item, "stopSignal", stop_signal, sizeof(stop_signal)) != 0 ||
	    strcmp(stop_signal, "SIGTERM") != 0 ||
	    json_uint(item, "restartBackoffMs", &child->backoff_ms, 100, 60000) != 0 ||
	    json_uint(item, "restartBackoffMaxMs", &child->backoff_max_ms,
		    child->backoff_ms, 300000) != 0 ||
	    json_uint(item, "restartLimit", &child->restart_limit, 0, 1000) != 0 ||
	    json_uint(item, "restartWindowSec", &child->restart_window_sec,
		    1, 3600) != 0 ||
	    json_uint(item, "stopTimeoutMs", &child->stop_timeout_ms,
		    1, 60000) != 0)
		return -1;

	value = cJSON_GetObjectItemCaseSensitive(item, "enabled");
	if (!cJSON_IsBool(value))
		return -1;
	enabled = cJSON_IsTrue(value);
	child->restart_on_failure = strcmp(restart, "on-failure") == 0;

	args = cJSON_GetObjectItemCaseSensitive(item, "argv");
	if (!cJSON_IsArray(args) || cJSON_GetArraySize(args) < 1 ||
	    cJSON_GetArraySize(args) >= CHILD_ARG_LIMIT)
		return -1;
	cJSON_ArrayForEach(value, args) {
		if (!cJSON_IsString(value) || !value->valuestring ||
		    strlen(value->valuestring) >= CHILD_TEXT_LIMIT)
			return -1;
		strcpy(child->args[child->argc++], value->valuestring);
	}

	env = cJSON_GetObjectItemCaseSensitive(item, "env");
	if (!cJSON_IsArray(env) || cJSON_GetArraySize(env) > CHILD_ENV_LIMIT)
		return -1;
	cJSON_ArrayForEach(entry, env) {
		if (!cJSON_IsString(entry) || !entry->valuestring ||
		    (strcmp(entry->valuestring, "PATH") != 0 &&
		     strcmp(entry->valuestring, "LD_LIBRARY_PATH") != 0 &&
		     strcmp(entry->valuestring, "HOME") != 0 &&
		     strcmp(entry->valuestring, "TZ") != 0))
			return -1;
		for (size_t i = 0; i < child->env_count; i++) {
			if (strcmp(child->env_names[i], entry->valuestring) == 0)
				return -1;
		}
		strcpy(child->env_names[child->env_count++], entry->valuestring);
	}
	return enabled ? 0 : 1;
}

static int parse_config(const char *json, size_t length, const char *vfrm_uri,
	ChildSpec *children, size_t *child_count)
{
	cJSON *root;
	cJSON *array;
	cJSON *item;
	size_t parsed_count = 0;
	size_t enabled_count = 0;
	int result = -1;

	if (!json || length == 0 || length > CHILD_JSON_LIMIT || !vfrm_uri ||
	    strncmp(vfrm_uri, "frame-shm://", 12) != 0)
		return -1;
	root = cJSON_ParseWithLength(json, length);
	if (!cJSON_IsObject(root))
		goto done;
	array = cJSON_GetObjectItemCaseSensitive(root, "children");
	if (!cJSON_IsArray(array) || cJSON_GetArraySize(array) > CHILD_LIMIT)
		goto done;
	cJSON_ArrayForEach(item, array) {
		int parsed = parse_child(item, &children[parsed_count]);
		if (parsed < 0)
			goto done;
		parsed_count++;
		if (parsed == 0)
			children[enabled_count++] = children[parsed_count - 1];
	}
	*child_count = enabled_count;
	result = 0;
done:
	cJSON_Delete(root);
	return result;
}

int child_supervisor_validate_json(const char *json, size_t length)
{
	ChildSpec children[CHILD_LIMIT];
	size_t child_count = 0;
	return parse_config(json, length, "frame-shm://venc0", children,
		&child_count);
}

static int append_fd_close_actions(posix_spawn_file_actions_t *actions)
{
	DIR *directory = opendir("/proc/self/fd");
	struct dirent *entry;
	int fds[CHILD_FD_LIMIT];
	size_t count = 0;
	int directory_fd;
	int result = 0;

	if (!directory)
		return -1;
	directory_fd = dirfd(directory);
	while ((entry = readdir(directory)) != NULL) {
		char *end;
		long fd;
		if (entry->d_name[0] < '0' || entry->d_name[0] > '9')
			continue;
		fd = strtol(entry->d_name, &end, 10);
		if (*end || fd <= STDERR_FILENO || fd == directory_fd)
			continue;
		if (count == CHILD_FD_LIMIT) {
			result = -1;
			break;
		}
		fds[count++] = (int)fd;
	}
	if (result == 0) {
		for (size_t i = 0; i < count; i++) {
			if (posix_spawn_file_actions_addclose(actions, fds[i]) != 0) {
				result = -1;
				break;
			}
		}
	}
	if (posix_spawn_file_actions_addclose(actions, directory_fd) != 0)
		result = -1;
	closedir(directory);
	return result;
}

static int spawn_child(ChildSupervisor *supervisor, ChildSpec *child)
{
	posix_spawn_file_actions_t actions;
	char *argv[CHILD_ARG_LIMIT + 1];
	char env_storage[CHILD_ENV_LIMIT + 1][CHILD_ENV_TEXT_LIMIT];
	char *envp[CHILD_ENV_LIMIT + 2];
	size_t env_count = 0;
	pid_t pid;
	int rc;

	for (size_t i = 0; i < child->argc; i++)
		argv[i] = child->args[i];
	argv[child->argc] = NULL;
	for (size_t i = 0; i < child->env_count; i++) {
		const char *value = getenv(child->env_names[i]);
		int written;
		if (!value)
			continue;
		written = snprintf(env_storage[env_count], sizeof(env_storage[env_count]),
			"%s=%s", child->env_names[i], value);
		if (written < 0 || (size_t)written >= sizeof(env_storage[env_count]))
			return -1;
		envp[env_count] = env_storage[env_count];
		env_count++;
	}
	int written = snprintf(env_storage[env_count], sizeof(env_storage[env_count]),
		"VFRM_URI=%s", supervisor->vfrm_uri);
	if (written < 0 || (size_t)written >= sizeof(env_storage[env_count]))
		return -1;
	envp[env_count] = env_storage[env_count];
	env_count++;
	envp[env_count] = NULL;

	rc = posix_spawn_file_actions_init(&actions);
	if (rc != 0)
		return -1;
	rc = append_fd_close_actions(&actions);
	if (rc == 0)
		rc = posix_spawn(&pid, child->executable, &actions, NULL, argv, envp);
	posix_spawn_file_actions_destroy(&actions);
	if (rc != 0) {
		errno = rc;
		fprintf(stderr, "[supervisor] spawn %s failed: %s\n", child->name,
			strerror(errno));
		return -1;
	}
	child->pid = pid;
	fprintf(stderr, "[supervisor] started %s pid=%ld\n", child->name,
		(long)pid);
	return 0;
}

static struct timespec add_ms(struct timespec time, unsigned int ms)
{
	time.tv_sec += ms / 1000;
	time.tv_nsec += (long)(ms % 1000) * 1000000L;
	if (time.tv_nsec >= 1000000000L) {
		time.tv_sec++;
		time.tv_nsec -= 1000000000L;
	}
	return time;
}

static int time_reached(struct timespec now, struct timespec deadline)
{
	return now.tv_sec > deadline.tv_sec ||
		(now.tv_sec == deadline.tv_sec && now.tv_nsec >= deadline.tv_nsec);
}

static void schedule_restart(ChildSpec *child, struct timespec now)
{
	unsigned int delay = child->backoff_ms;
	unsigned int shift = child->restarts > 1 ? child->restarts - 1 : 0;
	if (shift > 16)
		shift = 16;
	while (shift-- && delay < child->backoff_max_ms / 2)
		delay *= 2;
	if (delay > child->backoff_max_ms)
		delay = child->backoff_max_ms;
	child->restart_at = add_ms(now, delay);
}

static void stop_children(ChildSupervisor *supervisor)
{
	struct timespec now;
	struct timespec deadline = {0};
	int pending;

	clock_gettime(CLOCK_MONOTONIC, &now);
	for (size_t i = 0; i < supervisor->child_count; i++) {
		ChildSpec *child = &supervisor->children[i];
		if (child->pid > 0) {
			(void)kill(child->pid, SIGTERM);
			struct timespec child_deadline = add_ms(now,
				child->stop_timeout_ms);
			if (!deadline.tv_sec || time_reached(deadline, child_deadline))
				deadline = child_deadline;
		}
	}
	do {
		pending = 0;
		for (size_t i = 0; i < supervisor->child_count; i++) {
			ChildSpec *child = &supervisor->children[i];
			int status;
			if (child->pid <= 0)
				continue;
			if (waitpid(child->pid, &status, WNOHANG) == child->pid) {
				child->pid = 0;
				continue;
			}
			pending = 1;
		}
		if (!pending)
			break;
		clock_gettime(CLOCK_MONOTONIC, &now);
		if (time_reached(now, deadline))
			break;
		struct timespec pause = { .tv_sec = 0, .tv_nsec = 10000000L };
		nanosleep(&pause, NULL);
	} while (1);
	for (size_t i = 0; i < supervisor->child_count; i++) {
		ChildSpec *child = &supervisor->children[i];
		if (child->pid > 0) {
			(void)kill(child->pid, SIGKILL);
			while (waitpid(child->pid, NULL, 0) < 0 && errno == EINTR)
				;
			child->pid = 0;
		}
	}
}

static void *monitor_children(void *opaque)
{
	ChildSupervisor *supervisor = opaque;
	venc_frame_ring_t *ring = NULL;
	int ready = 0;
	for (;;) {
		struct timespec now;
		struct timespec wake;
		int stopping;

		pthread_mutex_lock(&supervisor->mutex);
		stopping = supervisor->stopping;
		pthread_mutex_unlock(&supervisor->mutex);
		if (stopping)
			break;
		clock_gettime(CLOCK_MONOTONIC, &now);
		if (!ring)
			ring = venc_frame_ring_attach(supervisor->vfrm_uri + 12);
		if (ring && !ready && __atomic_load_n(&ring->hdr->write_idx,
		    __ATOMIC_ACQUIRE) > 0) {
			ready = 1;
			for (size_t i = 0; i < supervisor->child_count; i++) {
				ChildSpec *child = &supervisor->children[i];
				child->window_start = now;
				if (spawn_child(supervisor, child) != 0 &&
				    child->restart_on_failure && child->restart_limit > 0) {
					child->restarts = 1;
					if (child->restarts >= child->restart_limit)
						child->restart_exhausted = 1;
					else
						schedule_restart(child, now);
				}
			}
		}
		for (size_t i = 0; ready && i < supervisor->child_count; i++) {
			ChildSpec *child = &supervisor->children[i];
			int status;
			pid_t result;
			if (child->pid > 0) {
				result = waitpid(child->pid, &status, WNOHANG);
				if (result == child->pid) {
					child->pid = 0;
					if (!child->restart_on_failure ||
					    (WIFEXITED(status) && WEXITSTATUS(status) == 0))
						continue;
					if (now.tv_sec - child->window_start.tv_sec >=
					    child->restart_window_sec) {
						child->restarts = 0;
						child->restart_exhausted = 0;
						child->window_start = now;
					}
					if (child->restarts >= child->restart_limit) {
						child->restart_exhausted = 1;
						fprintf(stderr, "[supervisor] restart limit reached for %s\n",
							child->name);
						continue;
					}
					child->restarts++;
					schedule_restart(child, now);
				}
			}
			if (child->pid == 0 && child->restart_on_failure &&
			    !child->restart_exhausted &&
			    child->restarts > 0 && time_reached(now, child->restart_at)) {
				if (spawn_child(supervisor, child) != 0 &&
				    child->restarts < child->restart_limit) {
					child->restarts++;
					if (child->restarts >= child->restart_limit)
						child->restart_exhausted = 1;
					else
						schedule_restart(child, now);
				} else if (child->restarts >= child->restart_limit) {
					child->restart_exhausted = 1;
				}
			}
		}
		clock_gettime(CLOCK_REALTIME, &wake);
		wake = add_ms(wake, CHILD_POLL_MS);
		pthread_mutex_lock(&supervisor->mutex);
		if (!supervisor->stopping)
			(void)pthread_cond_timedwait(&supervisor->condition,
				&supervisor->mutex, &wake);
		pthread_mutex_unlock(&supervisor->mutex);
	}
	stop_children(supervisor);
	if (ring)
		venc_frame_ring_destroy(ring);
	return NULL;
}

static int read_config(const char *path, char **out, size_t *length)
{
	struct stat st;
	struct stat opened;
	char *data;
	size_t used = 0;
	int fd;

	if (!path || path[0] != '/' || lstat(path, &st) != 0 ||
	    !S_ISREG(st.st_mode) || st.st_uid != 0 || (st.st_mode & 0022) ||
	    st.st_size <= 0 || st.st_size > CHILD_JSON_LIMIT) {
		errno = EACCES;
		return -1;
	}
	fd = open(path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
	if (fd < 0)
		return -1;
	if (fstat(fd, &opened) != 0 || !S_ISREG(opened.st_mode) ||
	    opened.st_uid != 0 || (opened.st_mode & 0022) ||
	    opened.st_size <= 0 || opened.st_size > CHILD_JSON_LIMIT) {
		close(fd);
		errno = EACCES;
		return -1;
	}
	st = opened;
	data = malloc((size_t)st.st_size + 1);
	if (!data) {
		close(fd);
		return -1;
	}
	while (used < (size_t)st.st_size) {
		ssize_t n = read(fd, data + used, (size_t)st.st_size - used);
		if (n <= 0) {
			free(data);
			close(fd);
			return -1;
		}
		used += (size_t)n;
	}
	close(fd);
	data[used] = '\0';
	*out = data;
	*length = used;
	return 0;
}

int child_supervisor_start(ChildSupervisor **out, const char *config_path,
	const char *vfrm_uri)
{
	ChildSupervisor *supervisor;
	char *json = NULL;
	size_t length = 0;

	if (!out || *out || !vfrm_uri || strlen(vfrm_uri) >= CHILD_TEXT_LIMIT ||
	    strncmp(vfrm_uri, "frame-shm://", 12) != 0 ||
	    read_config(config_path, &json, &length) != 0)
		return -1;
	supervisor = calloc(1, sizeof(*supervisor));
	if (!supervisor) {
		free(json);
		return -1;
	}
	strcpy(supervisor->vfrm_uri, vfrm_uri);
	if (parse_config(json, length, vfrm_uri, supervisor->children,
	    &supervisor->child_count) != 0) {
		free(supervisor);
		free(json);
		return -1;
	}
	free(json);
	if (supervisor->child_count == 0) {
		free(supervisor);
		return 0;
	}
	if (pthread_mutex_init(&supervisor->mutex, NULL) != 0) {
		free(supervisor);
		return -1;
	}
	if (pthread_cond_init(&supervisor->condition, NULL) != 0) {
		pthread_mutex_destroy(&supervisor->mutex);
		free(supervisor);
		return -1;
	}
	if (pthread_create(&supervisor->thread, NULL, monitor_children,
	    supervisor) != 0) {
		pthread_cond_destroy(&supervisor->condition);
		pthread_mutex_destroy(&supervisor->mutex);
		free(supervisor);
		return -1;
	}
	supervisor->thread_started = 1;
	*out = supervisor;
	return 0;
}

void child_supervisor_stop(ChildSupervisor **supervisor_ptr)
{
	ChildSupervisor *supervisor;
	if (!supervisor_ptr || !*supervisor_ptr)
		return;
	supervisor = *supervisor_ptr;
	*supervisor_ptr = NULL;
	if (supervisor->thread_started) {
		pthread_mutex_lock(&supervisor->mutex);
		supervisor->stopping = 1;
		pthread_cond_signal(&supervisor->condition);
		pthread_mutex_unlock(&supervisor->mutex);
		(void)pthread_join(supervisor->thread, NULL);
		pthread_cond_destroy(&supervisor->condition);
		pthread_mutex_destroy(&supervisor->mutex);
	}
	free(supervisor);
}
