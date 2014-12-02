#ifndef ENES100_RF_CLIENT_H_
#define ENES100_RF_CLIENT_H_

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "enes100_marker.h"

namespace enes100 {
  
template <class T>
class RfClient {
  private:
    T* _Serial;
    int _timeout_ms;
    char _marker_buffer[50];
  public:
    RfClient(T* _Serial) {
      this->_Serial = _Serial;
      this->_timeout_ms = 300;//Default 150ms msg request timeout
    };
    RfClient(T* _Serial, int timeout_ms) {
      this->_Serial = _Serial;
      this->_timeout_ms = timeout_ms;
    };
    uint8_t receiveMarker(Marker* m,int id){
      return this->receiveMarker(m, id, this->_timeout_ms);
    };
    uint8_t receiveMarker(Marker* m,int id, int timeout_ms){
      //Reset the state of the server.
      this->resetServer();
      //Flush client buffer.
      this->_Serial->flush();
      //Request marker from server.
      this->_Serial->print('#');
      this->_Serial->print(id);
      this->_Serial->print('*');
      //Attempt to fully receive the marker before timing out.
      long start_time = millis();
      uint8_t i = 0;
      while((millis()-start_time) < timeout_ms) {
        if(this->_Serial->available() > 0) {
          char c = this->_Serial->read();
          
          //Newline terminates the marker string.
          if(c != '\n') {
            this->_marker_buffer[i] = c;
            i++;
          }
          else {
            this->_marker_buffer[i] = 0;
            //Parse marker
            m->parseMarkerStringCSV(this->_marker_buffer);
            return 1;
          }
        }
      }
      this->resetServer();
      //The marker was not received before timing out, return false (0).
      return 0;
    };
    void resetServer(){
      this->_Serial->print('~');
    };
    void sendMessage(String msg){
      this->resetServer();
      this->_Serial->print('@');
      this->_Serial->print(msg);
      this->_Serial->print('*');
    };
};

}

#endif