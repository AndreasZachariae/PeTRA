#include <petra_central_control/default.h>

class test : public Component
{
private:
    /* data */
public:
    test(/* args */);
    ~test();
};

test::test(/* args */)
{
    log("test");
}

test::~test()
{
}



int main(int argc, char const *argv[])
{
    test t1;
    int c = 1;
    t1.start_chrono("timer");
    for (int i = 0; i < 10; i++)
    {
        c = c+i; 
    }
    t1.stop_chrono("timer");
    t1.log_chrono("timer");
    t1.log(std::to_string(c));
    return 0;
}
