#include "Button.h"

static void onButtonSingleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button single click");
}
static void onButtonDoubleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button double click");
}
static void onButtonLongPressStartCb(void *button_handle, void *usr_data) {
  Serial.println("Button long press click");
}


void setup_button(void){

  Button *btn1 = new Button(GPIO_NUM_3, false); //BUTTON_1_PIN
  Button *btn2 = new Button(GPIO_NUM_4, false); //BUTTON_2_PIN
  Button *btn3 = new Button(GPIO_NUM_7, false); //WIFI_RESET_PIN

  btn1->attachSingleClickEventCb(&onButtonSingleClickCb, NULL);
  btn1->attachDoubleClickEventCb(&onButtonDoubleClickCb, NULL);
  btn1->attachLongPressStartEventCb(&onButtonLongPressStartCb, NULL);

  btn2->attachSingleClickEventCb(&onButtonSingleClickCb, NULL);
  btn2->attachDoubleClickEventCb(&onButtonDoubleClickCb, NULL);
  btn2->attachLongPressStartEventCb(&onButtonLongPressStartCb, NULL);

  btn3->attachSingleClickEventCb(&onButtonSingleClickCb, NULL);
  btn3->attachDoubleClickEventCb(&onButtonDoubleClickCb, NULL);
  btn3->attachLongPressStartEventCb(&onButtonLongPressStartCb, NULL);

}