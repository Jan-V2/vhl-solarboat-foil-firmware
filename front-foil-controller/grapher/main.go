package main

import (
    "fmt"
    "log"
    "strings"
)

type Parser_State int
type Cmd string

const (
    start_state   Parser_State = 0
    home_wait = 1
    graph_wait  = 2
)

const(
    cmd_sep  Cmd = "|"
    cmd_home     = "HOME"
    cmd_graph    = "GRAPH"
    cmd_ack      = "ACK"
    cmd_done     = "DONE"
)



var parser_state = start_state
var serial_received string




func chk(err error){
    if err != nil {
        log.Fatal(err)
    }
}



func send_cmd(cmd Cmd, value int){
    switch cmd{
    case cmd_home:
        serial_write([]byte(cmd_home + cmd_sep))
        parser_state = home_wait
    case cmd_graph:
        cmd_str := fmt.Sprintf("%s %d%s",cmd_graph, value, cmd_sep)
        fmt.Print(cmd_str)
        serial_write([]byte(cmd_str))
        parser_state = graph_wait
    }
}

func check_done(cmd Cmd) bool{
    str := fmt.Sprintf("%s %s%s", cmd_done, cmd, cmd_sep)
    idx := strings.Index(serial_received , str)
    if  idx> -1{
        serial_received = serial_received[idx + len(str):]
        return true
        fmt.Print("done")
    }
    return false
}


func main() {
    serial_open()
    send_cmd(cmd_home, 0)

    buff := make([]byte, 100)
    for {
        n, _ := port.Read(buff)

        if n > 0 {
            serial_received += string(buff[:n])
            fmt.Print(string(buff[:n]))
            // parse the recieved data
            if parser_state == start_state{
                //strings.Index(serial_received, )
            }else if parser_state == home_wait{
                if check_done(cmd_home){
                    send_cmd(cmd_graph, -1000)
                }
            } else if parser_state == graph_wait {
                if check_done(cmd_graph){
                    send_cmd(cmd_home, 0)
                }
            }
        }

    }
}
