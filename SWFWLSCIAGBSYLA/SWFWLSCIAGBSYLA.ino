/*  Main arduino programm that controlls the Ferrovac GloveBox 
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


String versionString = "24900_v0.2.0_I";
//Compiled with LSClib v1.1.0_E


LSC& lsc = LSC::getInstance();
SceneManager& sceneManager = SceneManager::getInstance();
ComponentTracker& componentTracker = ComponentTracker::getInstance();


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


Persistent<bool> stateBufferIsVenting("bisv",false);
Persistent<bool> stateCryoDockIsVenting("cisv",false);
Persistent<bool> stateSuitCaseGVOpen("scgvo",false);
Persistent<bool> stateCooling("shc",true);


//Hardware Components
Components::RoughingPump pump_scrollPump(lsc.mosContact_0,"Scroll Pump");
Components::PressureGauge gauge_Buffer(lsc.analogInGauge_0,"Buffer Gauge", GaugeType::TPR); //PI1
Components::Valve valve_AngleBuffer(lsc.powerSwitch_1,"Buffer Angle Valve"); //VA03
Components::Valve valve_VentBuffer(lsc.powerSwitch_0,"Buffer Vent Valve"); //VA04

Components::PressureGauge gauge_CryoLoadLock(lsc.analogIn_3,"CLL Gauge", GaugeType::PKR); // PI02
Components::Valve valve_AngelCryoDock(lsc.openCollectorOutput_2,"CLL Angle Valve");
Components::RoughingPump pump_Turbo(lsc.mosContact_2, "Turbo Pump");
Components::TemperatureSensor temp_cryo(lsc.analogInPt100_1, "Pt100");

Components::GateValve gateValve_GloveBox(lsc.powerSwitch_2,lsc.powerSwitch_3,lsc.digitalInIsolated_5,"Gate Valve");

Components::Valve valve_LN2(lsc.openCollectorOutput_0,"LN2 Valve Dewar");
Components::LN2LevelMeter LN2LevelMeter(lsc.analogIn_0,"LN2 Level Sensor");


timer PumpLeakTimer;
Persistent<int> LN2_level("ln2l",35);
long LN2_Timer = 0;

unsigned long serialSendTime = 0;
unsigned long externalVentPressTimer = 0;
unsigned long bothAngleClosedTimer = 0;
bool externalVentPressed = false;
int externalVentCounter = 0;
bool externalVentDiasble = false;

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





void bufferScene(){
 
  SceneManager::UI_elements::VacuumChamber ui_loadLock(160,120,110,50);

  SceneManager::UI_elements::TextBox tb_pressure(115,115,"",FM9);
  SceneManager::UI_elements::TextBox tb_pressureUnit(130,115 + tb_pressure.getTextHeightInPixels(),"");
  SceneManager::UI_elements::GateValve ui_suitCaseGateValve(70,120,false,LinAlg::pi);
  SceneManager::UI_elements::GateValve ui_GloveBoxGateValve(260,120,false,LinAlg::pi);
  SceneManager::UI_elements::Valve ui_ventValve(110,70,false,LinAlg::pi);
  SceneManager::UI_elements::TextBox tb_ventGas(140,75, "N2");
  SceneManager::UI_elements::Valve ui_prePumpSeparationValve(235,80,true,LinAlg::pi*3./2.);
  SceneManager::UI_elements::Pump ui_scrollPump(235,26,true,LinAlg::pi*3./2.,0.8);
  SceneManager::UI_elements::VacuumChamber ui_uhvcts(0,120,80,120);
  SceneManager::UI_elements::VacuumChamber ui_glovebox(320,120,80,155);
  SceneManager::UI_elements::TextBox tb_name(320 /2 - 65/2,165,"Buffer",FMB9);
 


 // SceneManager::UI_elements::TextBox tb_systemTitle(50,185,"Log: ",FSSB9);
  SceneManager::UI_elements::TextBox tb_log(100 ,185,"",FSS9);
  if(OS::getBootUpState()) tb_log = "Reset because of hardFault";

  uint16_t tbUHVCTSXpos = 10;
  uint16_t tbUHVCTSYpos = 80;
  SceneManager::UI_elements::TextBox tb_u(tbUHVCTSXpos,tbUHVCTSYpos,"U",FMB9,TFT_WHITE);
  SceneManager::UI_elements::TextBox tb_h(tbUHVCTSXpos,tbUHVCTSYpos + tb_u.getTextHeightInPixels(),"H",FMB9,TFT_WHITE);
  SceneManager::UI_elements::TextBox tb_v(tbUHVCTSXpos,tbUHVCTSYpos + 2*tb_u.getTextHeightInPixels(),"V",FMB9,TFT_WHITE);
  SceneManager::UI_elements::TextBox tb_ccc(tbUHVCTSXpos,tbUHVCTSYpos + 3*tb_u.getTextHeightInPixels(),"C",FMB9,TFT_WHITE);
  SceneManager::UI_elements::TextBox tb_t(tbUHVCTSXpos,tbUHVCTSYpos + 4*tb_u.getTextHeightInPixels(),"T",FMB9,TFT_WHITE);
  SceneManager::UI_elements::TextBox tb_s(tbUHVCTSXpos,tbUHVCTSYpos + 5*tb_u.getTextHeightInPixels(),"S",FMB9,TFT_WHITE);
  uint16_t tbGLOVEBOXXpos = 300;
  uint16_t tbGLOVEBOXYpos = 63;
  SceneManager::UI_elements::TextBox tb_c(tbGLOVEBOXXpos,tbGLOVEBOXYpos,"G",FMB9,TFT_DARKGREY);
  SceneManager::UI_elements::TextBox tb_r(tbGLOVEBOXXpos,tbGLOVEBOXYpos + tb_c.getTextHeightInPixels(),"L",FMB9,TFT_DARKGREY);
  SceneManager::UI_elements::TextBox tb_y(tbGLOVEBOXXpos,tbGLOVEBOXYpos + 2*tb_c.getTextHeightInPixels(),"O",FMB9,TFT_DARKGREY);
  SceneManager::UI_elements::TextBox tb_o(tbGLOVEBOXXpos,tbGLOVEBOXYpos + 3*tb_c.getTextHeightInPixels(),"V",FMB9,TFT_DARKGREY);
  SceneManager::UI_elements::TextBox tb_d(tbGLOVEBOXXpos,tbGLOVEBOXYpos + 4*tb_c.getTextHeightInPixels(),"E",FMB9,TFT_DARKGREY);
  SceneManager::UI_elements::TextBox tb_oo(tbGLOVEBOXXpos,tbGLOVEBOXYpos + 5*tb_c.getTextHeightInPixels(),"B",FMB9,TFT_DARKGREY);
  SceneManager::UI_elements::TextBox tb_cc(tbGLOVEBOXXpos,tbGLOVEBOXYpos + 6*tb_c.getTextHeightInPixels(),"O",FMB9,TFT_DARKGREY);
  SceneManager::UI_elements::TextBox tb_k(tbGLOVEBOXXpos,tbGLOVEBOXYpos + 7*tb_c.getTextHeightInPixels(),"X",FMB9,TFT_DARKGREY);
  SceneManager::UI_elements::TextBox tb_settings(5,30,">Settings",FMB12,TFT_DARKGREY);
  SceneManager::UI_elements::Line uLBox1(0,45,tb_settings.getTextLengthInPixels()+10, 45,TFT_DARKGREY);
  SceneManager::UI_elements::Line uLBox2(tb_settings.getTextLengthInPixels()+10,45,tb_settings.getTextLengthInPixels()+10,0,TFT_DARKGREY);
  


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

  //SceneManager::UI_elements::TextBox tb_scrollPump(320 - SceneManager::tft.textWidth("<") -5,30,"<",FMB12,TFT_DARKGREY);

  SceneManager::UI_elements::TextBox tb_OpenClose(320 - SceneManager::tft.textWidth("Close GV<") -5,230," Open GV<",FMB12,TFT_DARKGREY);
  SceneManager::UI_elements::Line lRBox1(320 - tb_OpenClose.getTextLengthInPixels() - 20,205, 320, 205,TFT_DARKGREY);
  SceneManager::UI_elements::Line lRBox2(320 - tb_OpenClose.getTextLengthInPixels() - 20, 205,320 - tb_OpenClose.getTextLengthInPixels() - 20,240,TFT_DARKGREY);
  
  auto showOpenCloseButton = [&tb_OpenClose ,&lRBox1,&lRBox2](bool show = true){
    if(show){
      lRBox1.setColor(TFT_DARKGREY);
      lRBox2.setColor(TFT_DARKGREY);

    }else{
      lRBox1.setColor(sceneManager.getBackGroundColor());
      lRBox2.setColor(sceneManager.getBackGroundColor());
      tb_OpenClose = "";
    }
  };
  
  SceneManager::UI_elements::Line l1(ui_uhvcts.getRightConnectionPoint(), ui_suitCaseGateValve.getRightConnectionPoint());
  SceneManager::UI_elements::Line l2(ui_suitCaseGateValve.getLeftConnectionPoint(), ui_loadLock.getLeftConnectionPoint());
  SceneManager::UI_elements::Line l3(ui_ventValve.getRightConnectionPoint(), ui_suitCaseGateValve.getLeftConnectionPoint());
  SceneManager::UI_elements::Line l4(ui_loadLock.getRightConnectionPoint(),ui_GloveBoxGateValve.getRightConnectionPoint());
  SceneManager::UI_elements::Line l5(ui_prePumpSeparationValve.getRightConnectionPoint().vec[0],ui_prePumpSeparationValve.getRightConnectionPoint().vec[1],ui_prePumpSeparationValve.getRightConnectionPoint().vec[0], ui_loadLock.getRightConnectionPoint().vec[1]);
  SceneManager::UI_elements::Line l6(ui_prePumpSeparationValve.getLeftConnectionPoint(), ui_scrollPump.getLeftConnectionPoint());

  long unsigned int T = millis();

  while(!sceneManager.switchScene()){
    if(lsc.buttons.bt_4.hasBeenClicked()){
      sceneManager.loadScene(gloveBoxScene);
    }
    if(lsc.buttons.bt_0.isPressed()){
      sceneManager.showConfigMenu(versionString);
    }
    // VENT AND PUMP
    if(lsc.buttons.bt_2.hasBeenClicked()){
      if(stateBufferIsVenting){
        valve_AngelCryoDock.close(); //we need to protect the other chamber if it has good vac
        //delay(600)
        valve_AngleBuffer.open(1000);
        stateBufferIsVenting = false;
        continue;
      }
      if(gateValve_GloveBox.getState()) continue;
      if(stateSuitCaseGVOpen && !stateBufferIsVenting && !gateValve_GloveBox.getState()){
        if(sceneManager.showMessageBox("Warning", "Is the Suit Case gate valve Closed?", "No","Yes")){
          stateBufferIsVenting = true;
          stateSuitCaseGVOpen = false;
        } 
      }else{
        stateBufferIsVenting = true;
      }
    }
    //SHOW VENT PUMP BUTTON?
    if(stateBufferIsVenting){
      showVentButton(true);
      tb_vent = ">Pump";
    }else {
      if(gateValve_GloveBox.getState()){
        showVentButton(false);
      }else{
        showVentButton(true);
        tb_vent = ">Vent";
      }
    }

    //OPEN CLOSE
    if(lsc.buttons.bt_5.hasBeenClicked()){
      if(gateValve_GloveBox.getState()){
        if(sceneManager.showMessageBox("Grabber Retracted?", "Is the grabber fully retracted? Continuing while the grabber is not retracted will lead to the gate valve crashing into the transfer arm!","No","Yes")){
          if(!stateBufferIsVenting && !stateBufferIsVenting) gateValve_GloveBox.close();
        }
      }else{
        if(gauge_Buffer.getPressure() < 5 && gauge_CryoLoadLock.getPressure() < 5 && !stateBufferIsVenting && !stateCryoDockIsVenting){
          stateSuitCaseGVOpen = true;
          valve_AngelCryoDock.close();
          gateValve_GloveBox.open();
        }
      }
    }

    //SHOW OPEN CLOSE BUTTON?
    if(gateValve_GloveBox.getState()){
      showOpenCloseButton(true);
      tb_OpenClose = "Close GV<";
    }else{
      showVentButton(true);
      if(gauge_Buffer.getPressure() < 5 && gauge_CryoLoadLock.getPressure() < 5 && !stateBufferIsVenting && !stateCryoDockIsVenting){
        showOpenCloseButton(true);
        tb_OpenClose = " Open GV<";
      }else{
        showOpenCloseButton(false);
      }
    }

    //Set colour of pressure reading if error is present
    if(gauge_Buffer.error()){
      tb_pressure.setColor(TFT_RED);
    }else{
      tb_pressure.setColor(sceneManager.getForeGroundColor());
    }
    if(gauge_Buffer.getPressure() > 0.01){
      tb_pressure = gauge_Buffer.getPressureAsString(false);
      tb_pressureUnit = gauge_Buffer.getUnitSuffixAsString();
    }else{
      tb_pressure = "-URange-";
      tb_pressureUnit = "";
    }
    
    ui_ventValve.setState(valve_VentBuffer.getState());
    ui_GloveBoxGateValve.setState(gateValve_GloveBox.getState());
    ui_prePumpSeparationValve.setState(valve_AngleBuffer.getState());
    ui_scrollPump.setState(pump_scrollPump.getState());
    ui_suitCaseGateValve.setState(stateSuitCaseGVOpen);
    while(gauge_Buffer.error() && !gauge_Buffer.ignoreError()){
      valve_VentBuffer.setState(false);
      valve_AngleBuffer.setState(false);
      BEEPER.beep(3);
      if(sceneManager.showMessageBox("Gauge Error","A pressure gauge error has been detected. You can choose to ignore this error in the settings menu.","Okay","Settings")){
        sceneManager.showConfigMenu();
      }
    }
    mainControllFunction();
  }
}

void gloveBoxScene(){
  SceneManager::UI_elements::VacuumChamber ui_loadLock(160,120,110,50);
  SceneManager::UI_elements::VacuumChamber ui_Dewar(130,65,60,30,LinAlg::pi / 2.);
  int fillLineH = 65;
  SceneManager::UI_elements::Line ui_DewarFillLine(116,fillLineH,144,fillLineH);
    SceneManager::UI_elements::Line ui_DewarFillLine_v1(116 + 1 * 7.5 ,94,116 + 1 * 7.5,fillLineH);
    SceneManager::UI_elements::Line ui_DewarFillLine_v2(116 + 2 * 7.5 ,94,116 + 2 * 7.5,fillLineH);
    SceneManager::UI_elements::Line ui_DewarFillLine_v3(116 + 3 * 7.5 ,94,116 + 3 * 7.5,fillLineH);
  SceneManager::UI_elements::Valve ui_LN2_Valve(85,20,true,LinAlg::pi);
  SceneManager::UI_elements::TextBox tb_LN2(5,25,">LN2",FMB12,TFT_DARKGREY);

  SceneManager::UI_elements::TextBox tb_temp(150,65);

  SceneManager::UI_elements::TextBox tb_pressure(115,115,"",FM9);
  SceneManager::UI_elements::TextBox tb_name(160 - 87/2,165,"Glovebox",FMB9);
   

  SceneManager::UI_elements::TextBox tb_pressureUnit(130,115 + tb_pressure.getTextHeightInPixels(),"");
  SceneManager::UI_elements::GateValve ui_BufferGateValve(70,120,false,LinAlg::pi);

  SceneManager::UI_elements::Valve ui_CryoVentValve(300,50,true,LinAlg::pi/2.);
  SceneManager::UI_elements::TurboMolecularPump ui_Turbo(260,90,true,LinAlg::pi*3./2.,0.7);
  SceneManager::UI_elements::Valve ui_AngelValveToRoughingPump(260,50,true,LinAlg::pi*3./2.);
  
  SceneManager::UI_elements::Pump ui_scrollPump(220,20,true,0,0.7);

  SceneManager::UI_elements::VacuumChamber ui_Buffer(0,120,80,120);

  //SceneManager::UI_elements::TextBox tb_systemTitle(50,185,"Log: ",FSSB9);
  //SceneManager::UI_elements::TextBox tb_log(100 ,185,"System ready!",FSS9);
  

  uint16_t tbBUFFERXpos = 10;
  uint16_t tbBUFFERYpos = 80;
  SceneManager::UI_elements::TextBox tb_u(tbBUFFERXpos,tbBUFFERYpos,"B",FMB9,TFT_DARKGREY);
  SceneManager::UI_elements::TextBox tb_h(tbBUFFERXpos,tbBUFFERYpos + tb_u.getTextHeightInPixels(),"U",FMB9,TFT_DARKGREY);
  SceneManager::UI_elements::TextBox tb_v(tbBUFFERXpos,tbBUFFERYpos + 2*tb_u.getTextHeightInPixels(),"F",FMB9,TFT_DARKGREY);
  SceneManager::UI_elements::TextBox tb_ccc(tbBUFFERXpos,tbBUFFERYpos + 3*tb_u.getTextHeightInPixels(),"F",FMB9,TFT_DARKGREY);
  SceneManager::UI_elements::TextBox tb_t(tbBUFFERXpos,tbBUFFERYpos + 4*tb_u.getTextHeightInPixels(),"E",FMB9,TFT_DARKGREY);
  SceneManager::UI_elements::TextBox tb_s(tbBUFFERXpos,tbBUFFERYpos + 5*tb_u.getTextHeightInPixels(),"R",FMB9,TFT_DARKGREY);

  SceneManager::UI_elements::Line l1(ui_Buffer.getRightConnectionPoint(), ui_BufferGateValve.getRightConnectionPoint());
  SceneManager::UI_elements::Line l2(ui_BufferGateValve.getLeftConnectionPoint(), ui_loadLock.getLeftConnectionPoint());
  SceneManager::UI_elements::Line l3(ui_LN2_Valve.getLeftConnectionPoint().vec[0],ui_LN2_Valve.getLeftConnectionPoint().vec[1],ui_Dewar.getRightConnectionPoint().vec[0], ui_LN2_Valve.getLeftConnectionPoint().vec[1]);
  SceneManager::UI_elements::Line l4(ui_Dewar.getRightConnectionPoint().vec[0],ui_LN2_Valve.getLeftConnectionPoint().vec[1],ui_Dewar.getRightConnectionPoint().vec[0], ui_Dewar.getRightConnectionPoint().vec[1]);
  SceneManager::UI_elements::Line l5(ui_loadLock.getRightConnectionPoint().vec[0],ui_loadLock.getRightConnectionPoint().vec[1],ui_Turbo.getRightConnectionPoint().vec[0], ui_loadLock.getRightConnectionPoint().vec[1]);
  SceneManager::UI_elements::Line l6(ui_CryoVentValve.getRightConnectionPoint().vec[0],ui_CryoVentValve.getLeftConnectionPoint().vec[1], ui_CryoVentValve.getRightConnectionPoint().vec[0], 90);
  SceneManager::UI_elements::Line l9(260 + 17 , 90, ui_CryoVentValve.getRightConnectionPoint().vec[0], 90);
  SceneManager::UI_elements::TextBox tb_ventGas(290,20,"N2");

  SceneManager::UI_elements::Line l7(ui_Turbo.getRightConnectionPoint().vec[0],ui_loadLock.getRightConnectionPoint().vec[1], ui_Turbo.getRightConnectionPoint().vec[0], ui_Turbo.getLeftConnectionPoint().vec[1]);
  SceneManager::UI_elements::Line l8(ui_scrollPump.getLeftConnectionPoint().vec[0],ui_scrollPump.getRightConnectionPoint().vec[1], ui_AngelValveToRoughingPump.getRightConnectionPoint().vec[0], ui_scrollPump.getRightConnectionPoint().vec[1]);
  
  SceneManager::UI_elements::Line l10(ui_AngelValveToRoughingPump.getRightConnectionPoint().vec[0],ui_scrollPump.getRightConnectionPoint().vec[1], ui_AngelValveToRoughingPump.getRightConnectionPoint().vec[0], ui_AngelValveToRoughingPump.getLeftConnectionPoint().vec[1]);
    SceneManager::UI_elements::Line l11(ui_Dewar.getLeftConnectionPoint().vec[0],ui_Dewar.getLeftConnectionPoint().vec[1],ui_Dewar.getLeftConnectionPoint().vec[0],ui_Dewar.getLeftConnectionPoint().vec[1]-4, TFT_BLACK);

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

  SceneManager::UI_elements::TextBox tb_OpenClose(320 - SceneManager::tft.textWidth("Close GV<") -5,230," Open GV<",FMB12,TFT_DARKGREY);
  SceneManager::UI_elements::Line lRBox1(320 - tb_OpenClose.getTextLengthInPixels() - 20,205, 320, 205,TFT_DARKGREY);
  SceneManager::UI_elements::Line lRBox2(320 - tb_OpenClose.getTextLengthInPixels() - 20, 205,320 - tb_OpenClose.getTextLengthInPixels() - 20,240,TFT_DARKGREY);

  auto showOpenCloseButton = [&tb_OpenClose ,&lRBox1,&lRBox2](bool show = true){
    if(show){
      lRBox1.setColor(TFT_DARKGREY);
      lRBox2.setColor(TFT_DARKGREY);

    }else{
      lRBox1.setColor(sceneManager.getBackGroundColor());
      lRBox2.setColor(sceneManager.getBackGroundColor());
      tb_OpenClose = "";
    }
  };

  externalVentDiasble = !lsc.digitalInIsolated_1.getState();

  while(!sceneManager.switchScene()){
    if(lsc.buttons.bt_1.hasBeenClicked()){
      sceneManager.loadScene(bufferScene);
    }
    if(lsc.buttons.bt_0.hasBeenClicked()){
      sceneManager.loadScene(LN2Scene);
    }
    
    tb_temp = temp_cryo.getTeperatureAsString();

    tb_pressure = gauge_CryoLoadLock.getPressureAsString(false);
    tb_pressureUnit = gauge_CryoLoadLock.getUnitSuffixAsString();

    ui_scrollPump.setState(pump_scrollPump.getState());
    ui_AngelValveToRoughingPump.setState(valve_AngelCryoDock.getState());
    ui_Turbo.setState(pump_Turbo.getState());
    ui_LN2_Valve.setState(valve_LN2.getState());
    ui_CryoVentValve.setState( !pump_Turbo.getState());
    ui_BufferGateValve.setState(gateValve_GloveBox.getState());


    //OPEN CLOSE GATE VALVE
    if(lsc.buttons.bt_5.hasBeenClicked()){
      if(gateValve_GloveBox.getState()){
        if(sceneManager.showMessageBox("Grabber Retracted?", "Is the grabber fully retracted? Continuing while the grabber is not retracted will lead to the gate valve crashing into the transfer arm!","No","Yes")){
          if(!stateBufferIsVenting && !stateBufferIsVenting) gateValve_GloveBox.close();
        }
      }else{
        if(gauge_Buffer.getPressure() < 5 && gauge_CryoLoadLock.getPressure() < 5 && !stateBufferIsVenting && !stateCryoDockIsVenting){
          stateSuitCaseGVOpen = true;
          valve_AngleBuffer.close();
          gateValve_GloveBox.open();
        }
      }
    }
    //SHOW OPEN CLOSE BUTTON?
    if(gateValve_GloveBox.getState()){
      showOpenCloseButton(true);
      showVentButton(false);
      tb_OpenClose = "Close GV<";
    }else{
      showVentButton(true);
      if(gauge_Buffer.getPressure() < 5 && gauge_CryoLoadLock.getPressure() < 5 && !stateBufferIsVenting && !stateCryoDockIsVenting){
        showOpenCloseButton(true);
        tb_OpenClose = " Open GV<";
      }else{
        showOpenCloseButton(false);
      }
    }
    // Venting Pumping
    if(millis() - externalVentPressTimer > 1000 && !lsc.digitalInIsolated_1.getState()){
      if(externalVentCounter > 20){
        externalVentPressed = true;
        externalVentPressTimer = millis();
        externalVentCounter = 0;
      }else{
        externalVentCounter++;
      }
      
    }
    if(externalVentDiasble) externalVentPressed = false;
    if(lsc.buttons.bt_2.hasBeenClicked() || externalVentPressed){
      externalVentPressed = false;
      if(stateCryoDockIsVenting){
        valve_AngleBuffer.close(); //we need to protect Buffer vac 
        valve_AngelCryoDock.open(1000);
        stateCryoDockIsVenting = false;
      }else {
        if(!gateValve_GloveBox.getState()){ // Conditions to allow venting Cryo Dock
          stateCryoDockIsVenting = true;
        }
      }
    }

    //tb_LN2Level = String(LN2LevelMeter.getState());
    int fillLineH = 95 - (int)map(LN2LevelMeter.getState(),0,100,4,60);
    if(LN2LevelMeter.getState() > 3){
      ui_DewarFillLine.setPos(116, fillLineH, 144, fillLineH);
      ui_DewarFillLine_v1.setPos(116 + 1 * 7.5 ,95 ,116 + 1 * 7.5,fillLineH);
      ui_DewarFillLine_v2.setPos(116 + 2 * 7.5 ,95 ,116 + 2 * 7.5,fillLineH);
      ui_DewarFillLine_v3.setPos(116 + 3 * 7.5 ,95 ,116 + 3 * 7.5,fillLineH);
    }else {
      ui_DewarFillLine.clear();
      ui_DewarFillLine_v1.clear();
      ui_DewarFillLine_v2.clear();
      ui_DewarFillLine_v3.clear();
    }
    
    if(gauge_CryoLoadLock.error()){
      tb_pressure.setColor(TFT_RED);
    }else{
      tb_pressure.setColor(sceneManager.getForeGroundColor());
    }

    //Show vent Button?
    if(stateCryoDockIsVenting){
      showVentButton(true);
      tb_vent = ">Pump";
    }else{
      if(gateValve_GloveBox.getState()){
        showVentButton(false);
      }else{
        showVentButton(true);
        tb_vent = ">Vent";
      }
      
    }
    while(gauge_CryoLoadLock.error() && !gauge_CryoLoadLock.ignoreError()){
      BEEPER.beep(3);
      if(sceneManager.showMessageBox("Gauge Error","A pressure gauge error has been detected. You can choose to ignore this error in the settings menu.","Okay","Settings")){
        sceneManager.showConfigMenu();
      }
    }
    mainControllFunction();
  }
}

void LN2Scene(){
  LN2_level.setMinIntervall(0);
  LN2_level.readObjectFromSD();
  SceneManager::UI_elements::VacuumChamber ui_loadLock(340,260,400,120);
  SceneManager::UI_elements::VacuumChamber ui_Dewar(190,110,180,90,LinAlg::pi / 2.);
    SceneManager::UI_elements::Line l11(ui_Dewar.getLeftConnectionPoint().vec[0],ui_Dewar.getLeftConnectionPoint().vec[1],ui_Dewar.getLeftConnectionPoint().vec[0],ui_Dewar.getLeftConnectionPoint().vec[1]-4, TFT_BLACK);


  SceneManager::UI_elements::ScrollBar ui_scrollBar(TFT_DARKGREY);


  SceneManager::UI_elements::TextBox tb_transferRequest(5,30,">Back",FMB12,TFT_DARKGREY);
  SceneManager::UI_elements::Line uLBox1(0,45,tb_transferRequest.getTextLengthInPixels()+10, 45,TFT_DARKGREY);
  SceneManager::UI_elements::Line uLBox2(tb_transferRequest.getTextLengthInPixels()+10,45,tb_transferRequest.getTextLengthInPixels()+10,0,TFT_DARKGREY);
  
  SceneManager::UI_elements::Line ui_DewarFillLine(146,110,234,110,TFT_GREENYELLOW);
    int v1H = 110;
    SceneManager::UI_elements::Line ui_DewarFillLine_v1(146 + 14.666 * 1 ,199,146 + 14.666 * 1,v1H,TFT_GREENYELLOW);
    SceneManager::UI_elements::Line ui_DewarFillLine_v2(146 + 14.666 * 2 ,199,146 + 14.666 * 2,v1H,TFT_GREENYELLOW);
    SceneManager::UI_elements::Line ui_DewarFillLine_v3(146 + 14.666 * 3 ,199,146 + 14.666 * 3,v1H,TFT_GREENYELLOW);
    SceneManager::UI_elements::Line ui_DewarFillLine_v4(146 + 14.666 * 4 ,199,146 + 14.666 * 4,v1H,TFT_GREENYELLOW);
    SceneManager::UI_elements::Line ui_DewarFillLine_v5(146 + 14.666 * 5 ,199,146 + 14.666 * 5,v1H,TFT_GREENYELLOW);
  bool dewarFillLineCleard = false;
  SceneManager::UI_elements::Line ui_DewarFillLineMax(146,110,234,110,TFT_DARKGREEN);
  SceneManager::UI_elements::Line ui_DewarFillLineMin(146,110,234,110,TFT_GREEN);

  uint16_t stateXpos = 10;
  uint16_t stateYpos = 90;

  SceneManager::UI_elements::TextBox tb_temp(150,50,"",FMB12,TFT_WHITE);

  SceneManager::UI_elements::TextBox tb_LN2Titel(stateXpos,stateYpos,"LN2 Level",FMB12,TFT_WHITE);
  SceneManager::UI_elements::TextBox tb_LN2Max(stateXpos,stateYpos + 1 * tb_LN2Titel.getTextHeightInPixels(),"",FMB12,TFT_DARKGREEN);
  SceneManager::UI_elements::TextBox tb_LN2Min(stateXpos,stateYpos + 2 * tb_LN2Max.getTextHeightInPixels(),"",FMB12,TFT_GREEN);
  SceneManager::UI_elements::TextBox tb_LN2Current(stateXpos,stateYpos + 3 * tb_LN2Max.getTextHeightInPixels(),"",FMB12,TFT_GREENYELLOW);

  SceneManager::UI_elements::TextBox tb_HeatCool(5,230,">Fill On ",FMB12,TFT_DARKGREY);
  SceneManager::UI_elements::Line lLBox1(0,205,tb_HeatCool.getTextLengthInPixels()+20, 205,TFT_DARKGREY);
  SceneManager::UI_elements::Line lLBox2(tb_HeatCool.getTextLengthInPixels()+20,205,tb_HeatCool.getTextLengthInPixels()+20,240,TFT_DARKGREY);

  while(!sceneManager.switchScene()){
    if(lsc.buttons.bt_0.hasBeenClicked()){
      sceneManager.loadScene(gloveBoxScene);
    }
    if(lsc.buttons.bt_3.hasBeenClicked()){
      if(LN2_level <100) LN2_level += 1;
    }
    if(lsc.buttons.bt_4.hasBeenClicked()){
      if(LN2_level >2)LN2_level -= 1;
    }
    if(lsc.buttons.bt_2.hasBeenClicked()){
      if(stateCooling){
        stateCooling = false;
      }else {
        stateCooling = true;
      }
    }
    if(stateCooling){
      tb_HeatCool = ">Fill Off";
    }else{
      tb_HeatCool = ">Fill On";
    }

    tb_temp = temp_cryo.getTeperatureAsString();

    if(LN2LevelMeter.getState() > 5){
      int v1H = 200-(int)map(LN2LevelMeter.getState(),0,100,0,180);
      ui_DewarFillLine.setPos(146, v1H, 234, v1H );
      
      ui_DewarFillLine_v1.setPos(146 + 14.666 * 1 ,199,146 + 14.666 * 1,v1H);
      ui_DewarFillLine_v2.setPos(146 + 14.666 * 2 ,199,146 + 14.666 * 2,v1H);
      ui_DewarFillLine_v3.setPos(146 + 14.666 * 3 ,199,146 + 14.666 * 3,v1H);
      ui_DewarFillLine_v4.setPos(146 + 14.666 * 4 ,199,146 + 14.666 * 4,v1H);
      ui_DewarFillLine_v5.setPos(146 + 14.666 * 5 ,199,146 + 14.666 * 5,v1H);
      dewarFillLineCleard = false;
    }else {
      if(!dewarFillLineCleard){
        ui_DewarFillLine.clear();
        ui_DewarFillLine_v1.clear();
        ui_DewarFillLine_v2.clear();
        ui_DewarFillLine_v3.clear();
        ui_DewarFillLine_v4.clear();
        ui_DewarFillLine_v5.clear();
        dewarFillLineCleard = true;
      }
      
    }

    ui_DewarFillLineMax.setPos(146, 200-(int)map(LN2_level,0,100,0,177), 234, 200-(int)map(LN2_level,0,100,0,177));
    ui_DewarFillLineMin.setPos(146, 200-(int)map(LN2_level * 0.5,0,100,0,177), 234, 200-(int)map(LN2_level * 0.5,0,100,0,177));
    ui_DewarFillLineMax.reDraw();
    ui_DewarFillLineMin.reDraw();
    tb_LN2Min = "min: " + String(LN2_level * 0.5,0) + "%";
    tb_LN2Max = "max: "  + String(LN2_level)+ "%";
    tb_LN2Current = "is: " + String(LN2LevelMeter.getState())+ "%";
    mainControllFunction();
  }
}



void mainControllFunction(){

  //global condition glovebox gate valve
  if(gateValve_GloveBox.getState()){
    valve_AngleBuffer.close();
    valve_VentBuffer.close();
    valve_AngelCryoDock.open();
    pump_scrollPump.turnOn();
    pump_Turbo.turnOn();
  }

  //Buffer Venting Pumping
  if(stateBufferIsVenting){
    if(gateValve_GloveBox.getState()){
      stateBufferIsVenting = false;
      sceneManager.showMessageBox("Error Venting","Can not vent Buffer while Glove Box gate valce is open.", "", "Okay");
      return;
    }

    valve_AngleBuffer.close();
    valve_VentBuffer.open();
  }else{
    valve_VentBuffer.close();
    pump_scrollPump.turnOn();
  }

  //Cryo Dock Venting Pumping
  if(stateCryoDockIsVenting && !gateValve_GloveBox.getState()){
    valve_AngelCryoDock.close();
    pump_Turbo.turnOff();

  }
  if(!stateCryoDockIsVenting){
    pump_scrollPump.turnOn();
    pump_Turbo.turnOn();
  }

  if(stateBufferIsVenting && !stateCryoDockIsVenting){
    valve_AngleBuffer.close();
    valve_AngelCryoDock.open();
  }
  if(!stateBufferIsVenting && stateCryoDockIsVenting){
    valve_AngelCryoDock.close();
    valve_AngleBuffer.open();
  }

  if(!stateBufferIsVenting && !stateCryoDockIsVenting && !valve_AngelCryoDock.getState() && !valve_VentBuffer.getState()){
    if(millis() - bothAngleClosedTimer > 2000){
      valve_AngelCryoDock.open();
      bothAngleClosedTimer = millis();
    }
  }else {
    bothAngleClosedTimer = millis();
  }
  
  bool skipCycle = false;
  //Global conditions to open sep valves
  if(!stateBufferIsVenting && !stateCryoDockIsVenting  && gauge_Buffer.getPressure() < 50 && !gateValve_GloveBox.getState()){
    valve_AngelCryoDock.open();
    if(gauge_CryoLoadLock.getPressure() < 2) valve_AngleBuffer.open();
    skipCycle = true;
  }

  if(!stateBufferIsVenting && !stateCryoDockIsVenting && !skipCycle && gauge_Buffer.getPressure() > 3 && gauge_CryoLoadLock.getPressure() < 5 && !gateValve_GloveBox.getState()){
    valve_AngelCryoDock.close();
    valve_AngleBuffer.open();
  } 


  //LN2 Filling
  bool skipRest = false;
  if(stateCooling && (LN2LevelMeter.getState() < LN2_level)){
    if(LN2LevelMeter.getState() < LN2_level * 0.5){
        valve_LN2.open();
    }else{
      if(millis() - LN2_Timer > 60000 && (LN2LevelMeter.getState() < LN2_level)){
        valve_LN2.open();
        if(millis() -LN2_Timer > 61000){
          LN2_Timer = millis();
          valve_LN2.close();
        }
        
        skipRest = true;
      }
    }
    if(LN2LevelMeter.getState() >= LN2_level && !skipRest){
      valve_LN2.close();
    }
  }else{
    valve_LN2.close();
  }
  if(millis() - serialSendTime > 1000){
    waitForSaveReadWrite();
    lsc.println(String(millis()) + "; " + String(temp_cryo.getTemperature(),2) + "; " + String(gauge_CryoLoadLock.getPressure(),10) + "; " + String(gauge_Buffer.getPressure(),10) );
    serialSendTime = millis();
  }
}

void setup() {
  OS::init(versionString);
  Serial.println("os init complete");
  OS::startWatchdog();
  sceneManager.init(bufferScene,TFT_BLACK,TFT_WHITE,FM9);
  OS::stopWatchdog(); 
}

void loop() {
  stateBufferIsVenting.setMinIntervall(0);
  stateCryoDockIsVenting.setMinIntervall(0);
  stateSuitCaseGVOpen.setMinIntervall(0);
  stateCooling.setMinIntervall(0);
  stateCooling.readObjectFromSD();
  stateSuitCaseGVOpen.readObjectFromSD();
  stateBufferIsVenting.readObjectFromSD();
  stateBufferIsVenting.readObjectFromSD();
  pump_scrollPump.turnOn();
  if(!stateBufferIsVenting && !stateCryoDockIsVenting){
    valve_AngelCryoDock.open();
    valve_AngleBuffer.close();
  }
  sceneManager.begin();
}