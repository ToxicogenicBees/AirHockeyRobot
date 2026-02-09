#ifndef KEYLOG_H
#define KEYLOG_H

#include <functional>
#include <thread>
#include <mutex>
#include <map>

class KeyLog {
    private:
        static std::map<char, std::function<void()>> _actions;
        static std::thread _action_thread;
        static std::mutex _action_guard;

        static void _actOnInput();
    
    public:
        static void init();

        static void connect(char key, std::function<void()> func);

        static void disconnect(char key);

};

#endif
