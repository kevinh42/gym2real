#pragma once
#define STACK_SIZE 16
#include <sys/mman.h>
#include <sys/shm.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/types.h>

#include <yaml-cpp/yaml.h>
#include <map>

#define errExit(msg)        \
    do                      \
    {                       \
        perror(msg);        \
        exit(EXIT_FAILURE); \
    } while (0)

struct StateStoreConfig
{
    StateStoreConfig()
    {
        YAML::Node config = YAML::LoadFile("statestore.yaml");
        auto critical = config["stores"]["critical"];
        auto general = config["stores"]["general"];
        for (YAML::const_iterator it = critical.begin(); it != critical.end(); ++it)
        {
            critical_config.insert(std::pair<std::string, int>(it->first.as<std::string>(), it->second.as<int>()));
        }
        for (YAML::const_iterator it = general.begin(); it != general.end(); ++it)
        {
            general_config.insert(std::pair<std::string, int>(it->first.as<std::string>(), it->second.as<int>()));
        }
    }
    std::map<std::string, int> critical_config;
    std::map<std::string, int> general_config;
};

struct StateStore
{
    StateStore()
    {
        if (mlockall(MCL_FUTURE) == -1) //needs root
            errExit("mlockall");

        for (std::map<std::string, int>::iterator it = cfg_.critical_config.begin(); it != cfg_.critical_config.end(); ++it)
        {
            /* name of shared memory segment */
            const char *name = it->first.c_str();
            /* size (in bytes) of shared memory segment */
            const int SIZE = it->second;

            /* shared memory file descriptor */
            int shm_fd;

            /* pointer to shared memory object */
            void *ptr;

            /* create the shared memory object */
            shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
            if (shm_fd == -1)
                errExit("shm_open");

            /* configure the size of the shared memory object */
            ftruncate(shm_fd, SIZE);

            /* memory map the shared memory object */
            ptr = mmap(0, SIZE, PROT_WRITE, MAP_SHARED, shm_fd, 0);
            if (ptr == MAP_FAILED)
                errExit("mmap");
            
            if (close(shm_fd) == -1)
                errExit("close");
        }
    }

    ~StateStore()
    {
        for (std::map<std::string, int>::iterator it = cfg_.critical_config.begin(); it != cfg_.critical_config.end(); ++it)
        {
            /* name of shared memory segment */
            const char *name = it->first.c_str();

            if (shm_unlink(name) == -1)
                errExit("close");
        }
        if (munlockall() == -1)
            errExit("munlockall");
    }

    //rw=0 for read, rw=1 for write
    template <class T>
    int create_critical_buffer(const char *name, const int &rw, T *&ptr, size_t & size)
    {
        /* size (in bytes) of shared memory segment */
        size = cfg_.critical_config.at(name);
        int shm_fd = shm_open(name, O_RDWR, 0666);
        if (shm_fd == -1)
                return -1;
        void *p;
        if (rw == 0)
            p = mmap(0, size, PROT_READ, MAP_SHARED, shm_fd, 0);
        else if (rw == 1)
            p = mmap(0, size, PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (p == MAP_FAILED)
            return -1;
        size = size/sizeof(T);
        ptr = (T *)p;
        return 0;
    }

private:
    StateStoreConfig cfg_;
};
