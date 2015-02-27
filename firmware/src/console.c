#include <string.h>
#include <console.h>
#include <platform.h>
#include <seos.h>


/* TODO print all testable components
   TODO may want to make this a module */

/*
 * Returns true if the console should exit.
 * command:  The user-input command. Must be null-terminated.
 */
bool Console_handle_command(char *command)
{
    if (strcmp(command, "exit") == 0) {
        return false;
    } else if (strcmp(command, "halt") == 0) {
        OS_halt();
        /* Simulate a CPU wake */
        platWake();
    } else {
        char *token = strtok(command, " ");
        if (token == NULL) {
            OS_log(LOG_WARN, "Invalid command.");
            return true;
        }
        if (strcmp(token, "testnode") == 0) {
            token = strtok(NULL, " ");
            if (token == NULL) {
                OS_log(LOG_WARN, "Improper usage.  Usage:  testnode [node name] [sim filename]");
                return true;
            }
            struct task_t *task = OS_get_task(token);
            if (!task) {
                OS_log(LOG_WARN, "App not found.");
            } else {
                task->_APP_start_task(task);
                token = strtok(NULL, " ");
                OS_log(LOG_WARN, "WTH is a simulation file???");
            }
        } else {
            OS_log(LOG_WARN, "Command not found.");
        }
    }
    return true;
}
