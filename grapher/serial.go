package main

import (
    "go.bug.st/serial.v1"
    "log"
    "fmt"
    "time"
)

var port serial.Port

func serial_open() error {
    // TODO learn more about how error handeling works and improve this
    ports, err := serial.GetPortsList()
    chk(err)
    if len(ports) == 0 {
        log.Fatal("No serial ports found!")
    }
    fmt.Printf("opening port at: %v\n", ports[0])
    mode := &serial.Mode{
        BaudRate: 115200,
    }
    port, err = serial.Open(ports[0], mode)
    return err;
}

func serial_write(data []byte){
    //TODO this is really messy
    _, err := port.Write(data)
    if err != nil{
        fmt.Print("error while writing to serial, attempting reconnect")
        time.Sleep(2 * time.Second)
        err =serial_open()
        chk(err)
        serial_write(data)
    }
}