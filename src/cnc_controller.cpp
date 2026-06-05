#include "cnc_controller.h"
#include "bcm2835_pwm.h"
#include <bcm2835.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <algorithm>

static BCM2835PWM* g_pwm = nullptr;

CNCController::CNCController() 
    : m_currentLine(0), m_running(false), m_paused(false), 
      m_stopRequested(false), m_emergencyStop(false), m_homing(false),
      m_spindleSpeed(0), m_spindleOn(false) {
    
    m_stepPins[0] = X_STEP_PIN; m_stepPins[1] = Y_STEP_PIN;
    m_stepPins[2] = Z_STEP_PIN; m_stepPins[3] = A_STEP_PIN;
    
    m_dirPins[0] = X_DIR_PIN; m_dirPins[1] = Y_DIR_PIN;
    m_dirPins[2] = Z_DIR_PIN; m_dirPins[3] = A_DIR_PIN;
    
    m_limitPins[0] = X_LIMIT_PIN; m_limitPins[1] = Y_LIMIT_PIN;
    m_limitPins[2] = Z_LIMIT_PIN; m_limitPins[3] = A_LIMIT_PIN;
    
    m_stepsPerMM.x = STEPS_PER_MM_FINAL;
    m_stepsPerMM.y = STEPS_PER_MM_FINAL;
    m_stepsPerMM.z = STEPS_PER_MM_FINAL;
    m_stepsPerMM.a = STEPS_PER_MM_FINAL;
    
    for(int i = 0; i < 4; i++) m_endstops[i] = false;
}

CNCController::~CNCController() { shutdown(); }

bool CNCController::initialize() {
    if(!bcm2835_init()) return false;
    
    for(int i = 0; i < 4; i++) {
        bcm2835_gpio_fsel(m_stepPins[i], BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_fsel(m_dirPins[i], BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_write(m_stepPins[i], LOW);
        bcm2835_gpio_write(m_dirPins[i], LOW);
        
        bcm2835_gpio_fsel(m_limitPins[i], BCM2835_GPIO_FSEL_INPT);
        bcm2835_gpio_set_pud(m_limitPins[i], BCM2835_GPIO_PUD_UP);
    }
    
    g_pwm = new BCM2835PWM(SPINDLE_PWM_PIN);
    if(g_pwm->initialize()) {
        g_pwm->setFrequency(1000);
        g_pwm->disable();
    }
    
    m_running = true;
    m_stepperThread = std::thread(&CNCController::stepperThread, this);
    m_endstopThread = std::thread(&CNCController::monitorEndstops, this);
    m_statusMessage = "Ready";
    return true;
}

void CNCController::shutdown() {
    m_running = false;
    m_stopRequested = true;
    
    if(m_stepperThread.joinable()) m_stepperThread.join();
    if(m_endstopThread.joinable()) m_endstopThread.join();
    
    if(g_pwm) { delete g_pwm; g_pwm = nullptr; }
    bcm2835_close();
}

bool CNCController::moveTo(double x, double y, double z, double a, double feedrate) {
    if(m_emergencyStop) return false;
    std::lock_guard<std::mutex> lock(m_mutex);
    return moveRelative(x - m_currentPos.x, y - m_currentPos.y, 
                        z - m_currentPos.z, a - m_currentPos.a, feedrate);
}

bool CNCController::moveRelative(double dx, double dy, double dz, double da, double feedrate) {
    if(m_emergencyStop) return false;
    std::lock_guard<std::mutex> lock(m_mutex);
    
    int stepsX = std::abs(dx * m_stepsPerMM.x);
    int stepsY = std::abs(dy * m_stepsPerMM.y);
    int stepsZ = std::abs(dz * m_stepsPerMM.z);
    int stepsA = std::abs(da * m_stepsPerMM.a);
    int maxSteps = std::max({stepsX, stepsY, stepsZ, stepsA});
    
    if(maxSteps == 0) return true;
    
    double maxDistance = std::max({std::abs(dx), std::abs(dy), std::abs(dz), std::abs(da)});
    double timeNeeded = maxDistance / (feedrate / 60.0);
    int delayUs = (timeNeeded * 1000000) / maxSteps;
    if(delayUs < 50) delayUs = 50;
    
    bcm2835_gpio_write(m_dirPins[0], dx >= 0 ? HIGH : LOW);
    bcm2835_gpio_write(m_dirPins[1], dy >= 0 ? HIGH : LOW);
    bcm2835_gpio_write(m_dirPins[2], dz >= 0 ? HIGH : LOW);
    bcm2835_gpio_write(m_dirPins[3], da >= 0 ? HIGH : LOW);
    bcm2835_delayMicroseconds(10);
    
    for(int i = 0; i < maxSteps && !m_stopRequested && !m_emergencyStop; i++) {
        if(i < stepsX) { bcm2835_gpio_write(m_stepPins[0], HIGH); bcm2835_gpio_write(m_stepPins[0], LOW); }
        if(i < stepsY) { bcm2835_gpio_write(m_stepPins[1], HIGH); bcm2835_gpio_write(m_stepPins[1], LOW); }
        if(i < stepsZ) { bcm2835_gpio_write(m_stepPins[2], HIGH); bcm2835_gpio_write(m_stepPins[2], LOW); }
        if(i < stepsA) { bcm2835_gpio_write(m_stepPins[3], HIGH); bcm2835_gpio_write(m_stepPins[3], LOW); }
        bcm2835_delayMicroseconds(delayUs);
    }
    
    m_currentPos.x += dx; m_currentPos.y += dy;
    m_currentPos.z += dz; m_currentPos.a += da;
    return true;
}

bool CNCController::homeAxis(int axis) {
    if(m_emergencyStop) return false;
    std::lock_guard<std::mutex> lock(m_mutex);
    
    bcm2835_gpio_write(m_dirPins[axis], LOW);
    bcm2835_delayMicroseconds(10);
    
    while(bcm2835_gpio_lev(m_limitPins[axis]) == HIGH && !m_stopRequested && !m_emergencyStop) {
        bcm2835_gpio_write(m_stepPins[axis], HIGH);
        bcm2835_delayMicroseconds(500);
        bcm2835_gpio_write(m_stepPins[axis], LOW);
        bcm2835_delayMicroseconds(500);
    }
    
    bcm2835_gpio_write(m_dirPins[axis], HIGH);
    for(int i = 0; i < 100; i++) {
        bcm2835_gpio_write(m_stepPins[axis], HIGH);
        bcm2835_delayMicroseconds(500);
        bcm2835_gpio_write(m_stepPins[axis], LOW);
        bcm2835_delayMicroseconds(500);
    }
    
    switch(axis) {
        case 0: m_currentPos.x = 0; break;
        case 1: m_currentPos.y = 0; break;
        case 2: m_currentPos.z = 0; break;
        case 3: m_currentPos.a = 0; break;
    }
    return true;
}

bool CNCController::homeAll() {
    m_homing = true;
    for(int i = 0; i < 4; i++) homeAxis(i);
    m_homing = false;
    return true;
}

void CNCController::stop() { m_stopRequested = true; m_statusMessage = "Stopped"; }
void CNCController::emergencyStop() { m_emergencyStop = true; m_stopRequested = true; spindleOff(); m_statusMessage = "EMERGENCY STOP!"; }
void CNCController::pause() { m_paused = true; m_statusMessage = "Paused"; }
void CNCController::resume() { m_paused = false; m_statusMessage = "Running"; }

bool CNCController::loadGCodeFile(const std::string& filename) {
    std::ifstream file(filename);
    if(!file.is_open()) return false;
    
    m_gcodeBuffer.clear();
    std::string line;
    while(std::getline(file, line)) {
        size_t commentPos = line.find(';');
        if(commentPos != std::string::npos) line = line.substr(0, commentPos);
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        if(!line.empty()) m_gcodeBuffer.push_back(line);
    }
    file.close();
    m_currentLine = 0;
    return true;
}

bool CNCController::executeGCode() {
    if(m_gcodeBuffer.empty()) return false;
    m_stopRequested = false; m_emergencyStop = false; m_paused = false;
    m_statusMessage = "Executing G-code";
    
    while(m_currentLine < m_gcodeBuffer.size() && !m_stopRequested && !m_emergencyStop) {
        while(m_paused && !m_stopRequested && !m_emergencyStop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if(executeCommand(m_gcodeBuffer[m_currentLine])) {
            m_currentLine++;
            m_statusMessage = "Line " + std::to_string(m_currentLine) + "/" + std::to_string(m_gcodeBuffer.size());
        } else {
            m_statusMessage = "Error at line " + std::to_string(m_currentLine);
            return false;
        }
    }
    m_statusMessage = "Complete";
    return true;
}

bool CNCController::executeCommand(const std::string& command) {
    GCodeCommand cmd;
    std::istringstream iss(command);
    char letter;
    double value;
    
    iss >> cmd.code;
    cmd.code = toupper(cmd.code);
    
    while(iss >> letter >> value) {
        cmd.parameters.push_back(value);
    }
    
    switch(cmd.code) {
        case 'G':
            if(cmd.parameters.empty()) return false;
            switch((int)cmd.parameters[0]) {
                case 0: case 1:
                    if(cmd.parameters.size() >= 5) {
                        return moveTo(cmd.parameters[1], cmd.parameters[2], 
                                     cmd.parameters[3], cmd.parameters[4],
                                     cmd.parameters.size() > 5 ? cmd.parameters[5] : 1000);
                    }
                    break;
                case 4:
                    if(cmd.parameters.size() >= 2) {
                        std::this_thread::sleep_for(std::chrono::milliseconds((int)cmd.parameters[1]));
                        return true;
                    }
                    break;
                case 28: return homeAll();
                case 90: case 91: return true;
            }
            break;
        case 'M':
            if(cmd.parameters.empty()) return false;
            switch((int)cmd.parameters[0]) {
                case 3:
                    if(cmd.parameters.size() >= 2) setSpindleSpeed(cmd.parameters[1]);
                    spindleOn();
                    return true;
                case 5: spindleOff(); return true;
                case 30: m_stopRequested = true; return true;
            }
            break;
    }
    return true;
}

void CNCController::stepperThread() {
    while(m_running) std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void CNCController::monitorEndstops() {
    while(m_running) {
        for(int i = 0; i < 4; i++) {
            bool triggered = (bcm2835_gpio_lev(m_limitPins[i]) == LOW);
            if(triggered != m_endstops[i]) {
                m_endstops[i] = triggered;
                if(triggered && !m_homing && !m_emergencyStop) {
                    emergencyStop();
                    m_statusMessage = "Endstop " + std::to_string(i) + " triggered!";
                }
            }
        }
        bcm2835_delay(10);
    }
}

Position CNCController::getCurrentPosition() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_currentPos;
}

void CNCController::setSpindleSpeed(int rpm) {
    m_spindleSpeed = rpm;
    if(m_spindleOn && g_pwm) {
        g_pwm->setDutyCycle((rpm * 100.0) / SPINDLE_MAX_SPEED);
    }
}

void CNCController::spindleOn() {
    if(m_emergencyStop) return;
    m_spindleOn = true;
    if(g_pwm) {
        g_pwm->setDutyCycle((m_spindleSpeed * 100.0) / SPINDLE_MAX_SPEED);
        g_pwm->enable();
    }
}

void CNCController::spindleOff() {
    m_spindleOn = false;
    if(g_pwm) {
        g_pwm->setDutyCycle(0);
        g_pwm->disable();
    }
}
