#include "learning_gem5/hello_object.hh"
#include <iostream>
using namespace std;

HelloObject::HelloObject(HelloObjectParams *params) :
    SimObject(params), event(*this), latency(100), timesLeft(10)
{
   cout<<"Hello World! From a SimObject!"<<endl;
}

void
HelloObject::startup()
{
    schedule(event, latency);
}

void
HelloObject::processEvent()
{
    timesLeft--;
    cout<< "Hello world! Processing the event!"<<timesLeft<<"left\n"<<endl;

    if (timesLeft <= 0) {
        cout<< "Done firing!\n"<<endl;
    } else {
        schedule(event, curTick() + latency);
    }
}

HelloObject*
HelloObjectParams::create()
{
    return new HelloObject(this);
}
