/*  Main arduino programm that controlls the Ferrovac FerroLoader 
    Copyright (C) 2024 Ferrovac AG

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, version 3 of the License.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    NOTE: This specific version of the license has been chosen to ensure compatibility 
          with the SD library, which is an integral part of this application and is 
          licensed under the same version of the GNU General Public License.
*/
#include <Arduino.h>
#include <SPI.h>
#include "LscOS.h"
#include "LscComponents.h"
#include "LscHardwareAbstraction.h"
#include "LscSceneManager.h"
#include "LscPersistence.h"


String versionString = "24960_v1.0.0_G";
//Compilled with LSClib v0.1.0



LSC& lsc = LSC::getInstance();
SceneManager& sceneManager = SceneManager::getInstance();
ComponentTracker& componentTracker = ComponentTracker::getInstance();

enum struct States{
  BOOTUP,
  PUMPING,
  IDLE_PUMPING,
  VENTING,
  IDLE_VENTING,
  VENTED,
  IDLE_VENTED,
  TRANSFER_INIT,
  IN_TRANSFER,
  TRANSFER_COMPLETE
};
Persistent<States> state("state",States::BOOTUP);
States previousState = States::BOOTUP;

struct timer{
  private:
    unsigned long startTime;
    volatile bool timerStarted;
  public:
    timer() : timerStarted(false), startTime(millis()){
    }
    void start(){
      if(timerStarted) return;
      startTime = millis();
      timerStarted = true;
    }
    unsigned long getTime() const {
      if(!timerStarted) return 0;
      return millis() - startTime;
    }

    void stop(){
      timerStarted = false;
    }

    operator unsigned long() const {
      return getTime();
    }
};

Components::PressureGauge pressureGauge_loadLock(lsc.analogInGauge_0,"Load Lock Gauge", GaugeType::PKR);
Components::Valve valve_vent(lsc.powerSwitch_0,"Vent Valve");
Components::Valve valve_shutOff(lsc.powerSwitch_1,"Roughing Line Valve");
Components::GateValve gateValve_FIB(lsc.powerSwitch_2,lsc.powerSwitch_3,lsc.digitalInIsolated_5,"FIB Gate Valve");
Components::FIB FIB(lsc.mosContact_2,lsc.analogOutIsolated_0,lsc.digitalInIsolated_1,pressureGauge_loadLock,"FIB");
timer FIB_transferTimer;
Components::RoughingPump pump_scrollPump(lsc.mosContact_0,"Scroll Pump");

timer PumpLeakTimer;


void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      if (numTabs == 0)
        Serial.println("** Done **");
      return;
    }

    for (uint8_t i = 0; i < numTabs; i++)
      Serial.print('\t');

    Serial.print(entry.name());

    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }

    entry.close();
  }
}





void mainScreen(){
  state.setMinIntervall(0);
  state.readObjectFromSD();
  
  previousState = state;
  state = States::BOOTUP;
  SceneManager::UI_elements::VacuumChamber ui_loadLock(160,120,110,50);
  SceneManager::UI_elements::TextBox tb_pressure(115,115,"",FM9);
  SceneManager::UI_elements::TextBox tb_pressureUnit(130,115 + tb_pressure.getTextHeightInPixels(),"");
  SceneManager::UI_elements::GateValve ui_fibGateValve(70,120,false,LinAlg::pi);
  SceneManager::UI_elements::GateValve ui_uhvctmGateValve(260,120,false,LinAlg::pi);
  SceneManager::UI_elements::Valve ui_ventValve(110,70,false,LinAlg::pi);
  SceneManager::UI_elements::TextBox tb_ventGas(140,75, "N2");
  SceneManager::UI_elements::Valve ui_prePumpSeparationValve(235,70,true,LinAlg::pi*3./2.);
  SceneManager::UI_elements::Pump ui_scrollPump(270,26,true,LinAlg::pi*3./2.,0.8);
  SceneManager::UI_elements::VacuumChamber ui_fib(0,120,80,80);
  SceneManager::UI_elements::VacuumChamber ui_uhvctm(320,120,80,120);

  SceneManager::UI_elements::TextBox tb_systemTitle(5,185,"Log: ",FSSB9);
  SceneManager::UI_elements::TextBox tb_log(10 + tb_systemTitle.getTextLengthInPixels(),185,"System ready!",FSS9);
  if(OS::getBootUpState()) tb_log = "Reset because of hardFault";

  uint16_t tbFibXpos = 10;
  uint16_t tbFibYpos = 105;
  SceneManager::UI_elements::TextBox tb_f(tbFibXpos,tbFibYpos,"F");
  SceneManager::UI_elements::TextBox tb_i(tbFibXpos,tbFibYpos + tb_f.getTextHeightInPixels(),"I");
  SceneManager::UI_elements::TextBox tb_b(tbFibXpos,tbFibYpos + 2*tb_f.getTextHeightInPixels(),"B");
  uint16_t tbUhctmXpos = 300;
  uint16_t tbUhctmYpos = 80;
  SceneManager::UI_elements::TextBox tb_u(tbUhctmXpos,tbUhctmYpos,"U");
  SceneManager::UI_elements::TextBox tb_h(tbUhctmXpos,tbUhctmYpos + tb_f.getTextHeightInPixels(),"H");
  SceneManager::UI_elements::TextBox tb_v(tbUhctmXpos,tbUhctmYpos + 2*tb_f.getTextHeightInPixels(),"V");
  SceneManager::UI_elements::TextBox tb_c(tbUhctmXpos,tbUhctmYpos + 3*tb_f.getTextHeightInPixels(),"C");
  SceneManager::UI_elements::TextBox tb_t(tbUhctmXpos,tbUhctmYpos + 4*tb_f.getTextHeightInPixels(),"T");
  SceneManager::UI_elements::TextBox tb_m(tbUhctmXpos,tbUhctmYpos + 5*tb_f.getTextHeightInPixels(),"S");
  SceneManager::UI_elements::TextBox tb_transferRequest(5,30,">Start Transfer",FMB12,TFT_DARKGREY);
  SceneManager::UI_elements::Line* uLBox1 = new SceneManager::UI_elements::Line(0,45,tb_transferRequest.getTextLengthInPixels()+10, 45,TFT_DARKGREY);
  SceneManager::UI_elements::Line* uLBox2 = new SceneManager::UI_elements::Line(tb_transferRequest.getTextLengthInPixels()+10,45,tb_transferRequest.getTextLengthInPixels()+10,0,TFT_DARKGREY);
  
  auto showTransferButton = [&tb_transferRequest,&uLBox1,&uLBox2](bool show = true){
    if(show){
      uLBox1->setColor(TFT_DARKGREY);
      uLBox2->setColor(TFT_DARKGREY);

    }else{
      uLBox1->setColor(sceneManager.getBackGroundColor());
      uLBox2->setColor(sceneManager.getBackGroundColor());
      tb_transferRequest = "";
    }
  };

  SceneManager::UI_elements::TextBox tb_vent(5,230,">Vent",FMB12,TFT_DARKGREY);
  SceneManager::UI_elements::Line lLBox1(0,205,tb_vent.getTextLengthInPixels()+20, 205,TFT_DARKGREY);
  SceneManager::UI_elements::Line lLBox2(tb_vent.getTextLengthInPixels()+20,205,tb_vent.getTextLengthInPixels()+20,240,TFT_DARKGREY);

  auto showVentButton = [&tb_vent,&lLBox1,&lLBox2](bool show = true){
    if(show){
      lLBox1.setColor(TFT_DARKGREY);
      lLBox2.setColor(TFT_DARKGREY);

    }else{
      lLBox1.setColor(sceneManager.getBackGroundColor());
      lLBox2.setColor(sceneManager.getBackGroundColor());
      tb_vent = "";
    }
  };

  SceneManager::UI_elements::TextBox tb_scrollPump(320 - SceneManager::tft.textWidth("<") -5,30,"<",FMB12,TFT_DARKGREY);

  SceneManager::UI_elements::TextBox tb_settings(320 - SceneManager::tft.textWidth("Settings<") -5,230,"Settings<",FMB12,TFT_DARKGREY);
  SceneManager::UI_elements::Line lRBox1(320 - tb_settings.getTextLengthInPixels() - 20,205, 320, 205,TFT_DARKGREY);
  SceneManager::UI_elements::Line lRBox2(320 - tb_settings.getTextLengthInPixels() - 20, 205,320 - tb_settings.getTextLengthInPixels() - 20,240,TFT_DARKGREY);
  
  auto showSettingsButton = [&tb_settings,&lRBox1,&lRBox2](bool show = true){
    if(show){
      tb_settings = "Settings<";
      lRBox1.setColor(TFT_DARKGREY);
      lRBox2.setColor(TFT_DARKGREY);

    }else{
      tb_settings = "";
      lRBox1.setColor(sceneManager.getBackGroundColor());
      lRBox2.setColor(sceneManager.getBackGroundColor());
    }
  };
  
  SceneManager::UI_elements::Line l1(ui_fib.getRightConnectionPoint(), ui_fibGateValve.getRightConnectionPoint());
  SceneManager::UI_elements::Line l2(ui_fibGateValve.getLeftConnectionPoint(), ui_loadLock.getLeftConnectionPoint());
  SceneManager::UI_elements::Line l3(ui_ventValve.getRightConnectionPoint(), ui_fibGateValve.getLeftConnectionPoint());
  SceneManager::UI_elements::Line l4(ui_loadLock.getRightConnectionPoint(),ui_uhvctmGateValve.getRightConnectionPoint());
  SceneManager::UI_elements::Line l5(ui_prePumpSeparationValve.getRightConnectionPoint().vec[0],ui_prePumpSeparationValve.getRightConnectionPoint().vec[1],ui_prePumpSeparationValve.getRightConnectionPoint().vec[0], ui_loadLock.getRightConnectionPoint().vec[1]);
  SceneManager::UI_elements::Line l6(ui_prePumpSeparationValve.getLeftConnectionPoint(), ui_scrollPump.getLeftConnectionPoint());

  long unsigned int T = millis();

  while(!sceneManager.switchScene()){
    switch (state) {
      case States::BOOTUP:{
        if(gateValve_FIB.getState()){
          state = States::IN_TRANSFER;
          break;
        }
        if(previousState == States::BOOTUP){
          size_t numberOfBackChecks = std::min(state.getNumbersOfEntries(),(long unsigned int)5);
          for(size_t index = 1; index <= numberOfBackChecks; index++){
            States historyState = state[state.getNumbersOfEntries()-index];
            if(historyState != States::BOOTUP){
              state = historyState;
              break;
            }
          }
          if(state == States::BOOTUP) state = States::IDLE_PUMPING;
        }else{
          state = previousState;
        }

        
        break;
      }

      case States::PUMPING:{
        tb_scrollPump = "<";
        ui_uhvctmGateValve.setState(false);
        
        if(pressureGauge_loadLock.getPressure() > 50000 && !PumpLeakTimer) PumpLeakTimer.start();
      
        if(PumpLeakTimer >= 2000 && pressureGauge_loadLock.getPressure() > 50000){
          valve_shutOff.close();
          sceneManager.showMessageBox("Leak Detected", "A leak has been detected.","Okay", "");
          state = States::VENTED;
          PumpLeakTimer.stop();
          break;
        }
        if(PumpLeakTimer > 4000) PumpLeakTimer.stop();
        

        if(pressureGauge_loadLock.getPressure() <= 5){
          showTransferButton(true);
          tb_transferRequest = ">Start Transfer";
          if(lsc.buttons.bt_0.hasBeenClicked()){
            if(pressureGauge_loadLock.error() && pressureGauge_loadLock.ignoreError()){
              if(sceneManager.showMessageBox("Gauge Warning", "The pressure gauge is in an error state, which you have chosen to ignore in the settings. Continuing with the transfer could result in venting the FIB through the FerroLoader!","Abort","Continue")){
                state = States::TRANSFER_INIT;
                PumpLeakTimer.stop();
              }
              break;
            }else{
              state = States::TRANSFER_INIT;
              PumpLeakTimer.stop();
            }

          } 
        }else{
          showTransferButton(false);
        }

        tb_log = "Pumping";
        tb_vent = ">Vent";
        showVentButton(true);
        showSettingsButton(true);
        valve_vent.close();
        pump_scrollPump.turnOn();
        valve_shutOff.open();
        if(lsc.buttons.bt_2.hasBeenClicked()) {
          state = States::VENTING;
          PumpLeakTimer.stop();
        }
        if(lsc.buttons.bt_3.hasBeenClicked()) {
          state = States::IDLE_PUMPING;
          PumpLeakTimer.stop();
        }
        if(lsc.buttons.bt_5.hasBeenClicked()) sceneManager.showConfigMenu(versionString);
        break;
      }
      case States::IDLE_PUMPING:{
        tb_scrollPump = "<";
        tb_log = "Idle Pumping";
        tb_vent = ">Vent";
        showVentButton(true);
        showSettingsButton(true);
        showTransferButton(false);

        valve_vent.close();
        pump_scrollPump.turnOff();
        valve_shutOff.close();
        if(lsc.buttons.bt_2.hasBeenClicked()) state = States::IDLE_VENTING;
        if(lsc.buttons.bt_3.hasBeenClicked()) state = States::PUMPING;
        if(lsc.buttons.bt_5.hasBeenClicked()) sceneManager.showConfigMenu(versionString);
        break;
      }
      case States::VENTING:{
        tb_scrollPump = "<";
        tb_log = "Venting";
        tb_vent = ">Pump";
        showVentButton(true);
        showSettingsButton(false);
        showTransferButton(false);

        valve_shutOff.close();
        pump_scrollPump.turnOn();
        valve_vent.open();
        if(lsc.buttons.bt_2.hasBeenClicked()){
          state = States::PUMPING;
        } 
        if(lsc.buttons.bt_3.hasBeenClicked()) state = States::IDLE_VENTING;
        if(pressureGauge_loadLock.getPressure() >= 67000) state = States::VENTED;
        break;
      }
      case States::IDLE_VENTING:{
        tb_scrollPump = "<";
        tb_log = "Idle Venting";
        tb_vent = ">Pump";
        showVentButton(true);
        showSettingsButton(false);
        showTransferButton(false);

        valve_shutOff.close();
        pump_scrollPump.turnOff();
        valve_vent.open();
        if(lsc.buttons.bt_2.hasBeenClicked()) state = States::IDLE_PUMPING;
        if(lsc.buttons.bt_3.hasBeenClicked()) state = States::VENTING;
        if(pressureGauge_loadLock.getPressure() >= 67000) state = States::IDLE_VENTED;
        break;
      }
      case States::VENTED:{
        tb_scrollPump = "<";
        tb_log = "Vented";
        tb_vent = ">Pump";
        showVentButton(true);
        showSettingsButton(true);
        showTransferButton(false);

        valve_shutOff.close();
        valve_vent.close();
        pump_scrollPump.turnOn();
        if(lsc.buttons.bt_2.hasBeenClicked()) state = States::PUMPING;
        if(lsc.buttons.bt_3.hasBeenClicked()) state = States::IDLE_VENTED;
        if(lsc.buttons.bt_5.hasBeenClicked()) sceneManager.showConfigMenu(versionString);
        if(pressureGauge_loadLock.getPressure() < 67000) state = States::VENTING;
        break;
      }
      case States::IDLE_VENTED:{
        tb_scrollPump = "<";
        tb_log = "Vented";
        tb_vent = ">Pump";
        showVentButton(true);
        showSettingsButton(true);
        showTransferButton(false);

        valve_shutOff.close();
        valve_vent.close();
        pump_scrollPump.turnOff();
        if(lsc.buttons.bt_2.hasBeenClicked()) state = States::IDLE_PUMPING;
        if(lsc.buttons.bt_3.hasBeenClicked()) state = States::VENTING;
        if(lsc.buttons.bt_5.hasBeenClicked()) sceneManager.showConfigMenu(versionString);
        if(pressureGauge_loadLock.getPressure() < 67000) state = States::IDLE_VENTING;
        break;
      }
      case States::TRANSFER_INIT:{
        tb_transferRequest = ">Abort Transfer";
        tb_scrollPump = "";
        valve_vent.close();
        pump_scrollPump.turnOn();
        valve_shutOff.open();

        showTransferButton(true);
        if(!FIB_transferTimer) FIB.sendTransferRequest();
        FIB_transferTimer.start();
        if(FIB_transferTimer > 24000){
          BEEPER.beep(1);
          if(sceneManager.showMessageBox("FIB Timeout", "The FIB did not respond to the transfer request in time.","Okay", "Retry")){
            FIB_transferTimer.stop();
            
          }else{
            FIB_transferTimer.stop();
            state = States::PUMPING;
          }
        }
        if(FIB_transferTimer){
          tb_log = "Transfer Request Sent... " + String(24 - FIB_transferTimer/1000);
        }
        if(lsc.buttons.bt_0.hasBeenClicked()){
          FIB_transferTimer.stop();
          state = States::PUMPING;
        }
        if(FIB.getRequestState() && pressureGauge_loadLock.getPressure() < 5){
          FIB_transferTimer.stop();
          state = States::IN_TRANSFER;
        }

        if(pressureGauge_loadLock.getPressure() > 6){
          BEEPER.beep(3);
          sceneManager.showMessageBox("Pressure Warning", "A pressure rise has been detected. The transfer has been aborted to protect the FIB!","Okay","");
          FIB_transferTimer.stop();
          state = States::PUMPING;
          break;
        }
        showSettingsButton(false);
        showVentButton(false);
        
        break;
      }
      case States::IN_TRANSFER:{
        tb_log = "In Transfer";
        showSettingsButton(false);
        showVentButton(false);
        showTransferButton(true);
        tb_transferRequest = ">End Transfer";
        tb_scrollPump = "";
        valve_vent.close();
        valve_shutOff.close();
        pump_scrollPump.turnOn();
        gateValve_FIB.open();
        if(lsc.buttons.bt_0.hasBeenClicked()){
          if(sceneManager.showMessageBox("Grabber Retracted?", "Is the grabber fully retracted? Continuing while the grabber is not retracted will lead to the gate valve crashing into the transfer arm!","No","Yes")){
            if(sceneManager.showMessageBox("Close Gate Valve", "Please manually close the suit case gate valve.", "Cancel", "Okay")) state = States::TRANSFER_COMPLETE;
          }
        }
        showTransferButton(true);
        if(!ui_uhvctmGateValve.getState()){
          if(sceneManager.showMessageBox("Open Gate Valve", "Please manually open the suitcase gate valve.", "", "Okay")) ui_uhvctmGateValve.setState(true);
        }
        break;
      }
      case States::TRANSFER_COMPLETE:{
        tb_log = "Transfer Complete";
        showSettingsButton(false);
        showVentButton(false);
        showTransferButton(true);
        gateValve_FIB.close();
        pump_scrollPump.turnOn();
        valve_vent.close();
        if(!gateValve_FIB.getState()) state = States::PUMPING;
        
        break;
      }
      default:{
        state = States::PUMPING;
        break;
      }
    }
    //This section will be executed no mattet which state the system is in
    

    //tb_pressure = pressureGauge_loadLock.getPressureAsString(false);
    if(millis() - T > 100){
      tb_pressure = pressureGauge_loadLock.getPressureAsString(false);
      T = millis();
    }
    
    if(pressureGauge_loadLock.error()){
      tb_pressure.setColor(TFT_RED);
    }else{
      tb_pressure.setColor(sceneManager.getForeGroundColor());
    }
    tb_pressureUnit = pressureGauge_loadLock.getUnitSuffixAsString();
    ui_ventValve.setState(valve_vent.getState());
    ui_fibGateValve.setState(gateValve_FIB.getState());
    ui_prePumpSeparationValve.setState(valve_shutOff.getState());
    ui_scrollPump.setState(pump_scrollPump.getState());
    while(pressureGauge_loadLock.error() && !pressureGauge_loadLock.ignoreError()){
      valve_vent.setState(false);
      valve_shutOff.setState(false);
      BEEPER.beep(3);
      if(sceneManager.showMessageBox("Gauge Error","A pressure gauge error has been detected. You can choose to ignore this error in the settings menu.","Okay","Settings")){
        sceneManager.showConfigMenu(versionString);
      }
    }


  }
}


void setup() {
  
  OS::init(versionString);
  Serial.println("os init complete");
  OS::startWatchdog();
  sceneManager.init(mainScreen,TFT_BLACK,TFT_WHITE,FM9);
  OS::stopWatchdog(); 
  

}

void loop() {
  sceneManager.begin();
}