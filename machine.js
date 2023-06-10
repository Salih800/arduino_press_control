let net = require('net');
let sql = require('../db.js');
require('log-timestamp');

let tcpServer = net.createServer(function (socket) {

});

tcpServer.listen(8181);
let tcpClients = {};
let socketIdAI = 0;

function onMessage(socket, message) {
    console.log(socket.id + " - TCP -> " + message);
    let paramIndex = message.indexOf(":");
    let param = message.slice(0, paramIndex).trim();
    let value = message.slice(paramIndex + 1).trim();
    if (!socket.imei) {
        if (param !== 'IMEI') {
            socket.destroy();
        }
        console.log("param : " + param);
        console.log("value : " + value);

        sql.dbGetFirst("SELECT user_id,imei FROM container WHERE imei = ?", [value]).then(res => {
            if (res) {
                socket.write('OK:1\n');
                for (let prob in tcpClients) {
                    if (tcpClients[prob].imei.toString() === value.toString()) {
                        onDisconnected(tcpClients[prob]);
                        console.log("Same imei already connected!");
                    }
                }
                console.log("SEND OK!");
                socket.imei = value;
                socket.isAlive = true;
                socket.user = res.user_id;
                tcpClients[socket.id] = socket;
                sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
                    [socket.user, param, value, socket.remoteAddress]);
                sendAllWsClients(socket.user,"CONNECTED")
            } else {
                console.log(socket.id + " - WRONG IMEI!");
                onDisconnected(socket);
            }
        });
        return;
    }
    if (param === 'K') {
        socket.isAlive = true;
        sql.dbi("UPDATE container SET last_contact = CURRENT_TIMESTAMP WHERE user_id = ?", [socket.user]);
    } else if (param === 'DOOR') {
        if (value === '1') {
            sql.dbi("UPDATE container SET is_open = 1,last_opened=CURRENT_TIMESTAMP WHERE user_id = ?", [socket.user]);
        } else if (value === '0') {
            sql.dbi("UPDATE container SET is_open = 0 WHERE user_id = ?", [socket.user]);
        }
        sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
            [socket.user, param, value, socket.remoteAddress])
    } else if (param === 'PRESS') {
        let ratio = value.split(',')[0];
        let isFull = value.split(',')[1];
        sql.dbi("UPDATE container SET current_fullness = ?,is_full=?,total_press=total_press+1 WHERE user_id = ?",
            [ratio, isFull, socket.user]);
        sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
            [socket.user, param, value, socket.remoteAddress])
    } else if (param === 'TEMP') {
        let temp1 = value.split(',')[0];
        let temp2 = value.split(',')[1];
        sql.dbi("UPDATE container SET current_temperature = ?,current_temperature2=? WHERE user_id = ?",
            [temp1, temp2, socket.user]);
        sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
            [socket.user, param, value, socket.remoteAddress])
    } else if (param === 'LID') {
        if (value === '1') {
            sql.dbi("UPDATE container SET is_lid_open = 1 WHERE user_id = ?", [socket.user]);
            setTimeout(() => {
                console.log("LID IS STILL OPEN FOR CLIENT = " + socket.user);
            }, 15000);

        } else if (value === '0') {
            sql.dbi("UPDATE container SET is_lid_open = 0 WHERE user_id = ?", [socket.user]);
        }
        sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
            [socket.user, param, value, socket.remoteAddress])
    } else if (param === 'BUTTON1') {
        console.log("param : " + param);
        console.log("value : " + value);
        sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
            [socket.user, param, value, socket.remoteAddress])
    } else if (param === 'BUTTON2') {
        console.log("param : " + param);
        console.log("value : " + value);
        sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
            [socket.user, param, value, socket.remoteAddress])
    } else if (param === 'GPS') {
        console.log("param : " + param);
        console.log("value : " + value);
        let gpsArray = value.split(",");
        if (gpsArray.length < 2) {
            console.log("GPS format is wrong => " + value);
            return;
        }
        let firstStatus = gpsArray[0];
        let secondStatus = gpsArray[1];
        if (secondStatus.toString() === "0") {
            return
        }
        let gpsTime = gpsArray[2];
        let lat = gpsArray[3];
        let lng = gpsArray[4];
        console.log('gpsTime => ' + gpsTime);
        console.log('gpsTime => ' + gpsTime);
        console.log('lat => ' + lat);
        console.log('lng => ' + lng);
        sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
            [socket.user, param, value, socket.remoteAddress])
        sql.dbi("UPDATE address SET address_lat = ?,address_lng=? WHERE address_user_id = ?", [lat, lng, socket.user]);
    } else if (param === 'BAT_V') {
        sql.dbi(`UPDATE container
                 SET current_voltage = ?
                 WHERE user_id = ?`, [value, socket.user]);
        sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
            [socket.user, param, value, socket.remoteAddress])
    } else if (param === 'SOL_V') {
        sql.dbi(`UPDATE container
                 SET current_solar_voltage = ?
                 WHERE user_id = ?`, [value, socket.user]);
        sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
            [socket.user, param, value, socket.remoteAddress])
    } else if (param === 'LED') {
        sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
            [socket.user, param, value, socket.remoteAddress])
    } else if (param === 'FULL') {
        // TAMAMEN DOLU
        // RESET:1 GONDER TEKRAR BASLATMAK ICIN
        sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
            [socket.user, param, value, socket.remoteAddress])
    } else if (param === 'DEBUG') {
        let clients = [];
        for (let prob in tcpClients) {
            clients.push({
                id: tcpClients[prob].id,
                user: tcpClients[prob].user
            })
        }
        socket.write(JSON.stringify(clients) + '\n');
    }
    //GPSPERIOD:3-1000
    //GPSC:1
    //TEMP_ALARM:1
    sendAllWsClients(socket.user,message)
    console.log("FROM " + socket.user + " RECEIVED " + message);
    let isV6 = socket.remoteFamily == 'IPv6' ? 1 : 0;
    sql.dbi("INSERT INTO tcp_log (ip,isV6,message) VALUES (?,?,?)", [
        socket.remoteAddress,
        isV6,
        message.toString()
    ]);
}

function onDisconnected(socket) {

    sql.dbi("INSERT INTO tcp_log (ip,isV6,message) VALUES (?,?,?)", [
        socket.remoteAddress,
        socket.remoteFamily == 'IPv6' ? 1 : 0,
        'DISCONNECTED'
    ]);
    if (socket.user) {
        sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
            [socket.user, "SERVER", "DISCONNECTED", socket.remoteAddress]);
        sendAllWsClients(socket.user,"DISCONNECTED")
    }

    console.log(socket.id + ' disconnected !');
    socket.destroy();
    delete tcpClients[socket.id];
    socket = null;
}

tcpServer.on('connection', function (socket) {
    socketIdAI++;
    console.log(socketIdAI + ' connected !');

    sql.dbi("INSERT INTO tcp_log (ip,isV6,message) VALUES (?,?,?)", [
        socket.remoteAddress,
        socket.remoteFamily == 'IPv6' ? 1 : 0,
        'CONNECTED'
    ]);
    socket.id = socketIdAI;
    socket.imei = null;
    // The server can also receive data from the client by reading from its socket.
    socket.on('data', function (chunk) {
        onMessage(socket, chunk.toString());
    });

    // When the client requests to end the TCP connection with the server, the server
    // ends the connection.
    socket.on('end', function () {
        onDisconnected(socket);
    });

    // Don't forget to catch error, for your own sake.
    socket.on('error', function (err) {
        console.log(`Error: ${err}`);
    });
});

const interval = setInterval(function ping() {
    for (let prob in tcpClients) {
        if (tcpClients[prob].isAlive === false) {
            sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
                [tcpClients[prob].user, "SERVER", "TIMEOUT", tcpClients[prob].remoteAddress]);
            onDisconnected(tcpClients[prob]);
            return;
        }
        tcpClients[prob].isAlive = false;
        //tcpClients[prob].write('K:1\n');
    }
}, 120000);

tcpServer.on('close', function close() {
    clearInterval(interval);
});


/////// WEBSOCKET
nextClientID = 0;
wsClients = {};
const WebSocket = require('ws');
const wsServer = new WebSocket.Server({port: 8282});
wsServer.on('connection', function connection(wsClient) {
    nextClientID++;
    console.log(nextClientID + ' WS connected !');
    wsClient.id = nextClientID;
    wsClients[wsClient.id] = wsClient;

    wsClient.isAlive = true;
    wsClient.on('pong', function he() {
        wsClient.isAlive = true;
    });

    wsClient.result = function (action, params) {
        this.send(JSON.stringify(
            {"action": action, "params": params}
        ));
    }

    wsClient.on('message', function incoming(message) {
        let json = JSON.parse(message);
        if (!('action' in json && 'params' in json)) {
            wsClient.result("error", "MISSING PARAMS!");
            wsClient.terminate();
            return;
        }

        if (json['action'] === 'GET_TCP_CLIENTS') {
            let clients = [];
            for (let prob in tcpClients) {
                clients.push({
                    id: tcpClients[prob].id,
                    user: tcpClients[prob].user
                })
            }
            wsClient.result("TCP_CLIENTS", clients);
        } else if (json['action'] === 'SEND_MESSAGE') {
            let id = parseInt(json['params']['id']);
            let message = json['params']['message'];
            tcpClients[id].write(message + '\n');
        }

        console.log('Websocket message: %d %s', this.id, message);
    });

    wsClient.on('error', function incoming(message) {
        console.log('Websocket error: %s', message);
    });
    wsClient.on('close', function incoming(code, reason) {
        onWsDisconnected(wsClient);
    });
});

function sendAllWsClients(user,message) {
  wsServer.clients.forEach(function each(wsClient) {
        wsClient.result("MESSAGE", {
            user:user,
            message:message
        });
    });
}

function wsNoop() {
}

const wsInterval = setInterval(function ping() {
    wsServer.clients.forEach(function each(wsClient) {
        if (wsClient.isAlive === false) return wsClient.terminate();
        wsClient.isAlive = false;
        wsClient.ping(wsNoop);
    });
}, 30000);

wsServer.on('close', function close() {
    clearInterval(wsInterval);
});

function onWsDisconnected(socket) {
    console.log(socket.id + ' ws disconnected !');
    delete wsClients[socket.id];
}