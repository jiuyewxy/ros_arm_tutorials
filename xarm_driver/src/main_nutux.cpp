#include <iostream>                // std::cout
#include <thread>                // std::thread
#include <mutex>                // std::mutex, std::unique_lock
#include <condition_variable>    // std::condition_variable
#include <ros/ros.h>
std::mutex mtx;
std::condition_variable cv;

bool stop_request = false;

void requestData(){
  while(true){
    std::unique_lock <std::mutex> lck(mtx);
    if (!cv.wait_for(lck, std::chrono::milliseconds(100), [&] { return !stop_request; }))
      continue;
    std::cout <<"request data"<<std::endl;
    sleep(1);
  }
}

void cut(){
  sleep(5);
  mtx.try_lock();
  stop_request = true;
  std::cout << "cut test"<<std::endl;

  //sleep(2);
  std::cout << "restart ..."<<std::endl;
  mtx.unlock();
  stop_request = false;
  cv.notify_one();

  sleep(5);
  mtx.try_lock();
  stop_request = true;
  std::cout << "cut test"<<std::endl;

  //sleep(2);
  std::cout << "restart ..."<<std::endl;
  mtx.unlock();
  stop_request = false;
  cv.notify_one();

}

int main()
{
  std::thread tj_thread_ = std::thread(requestData);

  std::thread cut_thread = std::thread(cut);
  mtx.unlock();
  cv.notify_one();
  tj_thread_.join();
  cut_thread.join();
  return 0;
}
