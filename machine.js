let net = require('net');
let sql = require('../db.js');
require('log-timestamp');

let tcpServer = net.createServer(function (socket) {

});

tcpServer.listen(8181);
let tcpClients = {};
let socketIdAI = 0;

function onMessage(socket, message) {
    // console.log(socket.id + " - TCP -> " + message);
    if (!socket.imei) {
        if (!message.startsWith('IMEI:')) {
            console.log("IMEI is not first message! Destroying socket! Message => " + message);
            socket.destroy();
            return;
        }
        let imei = message.split(":")[1].trim();

        sql.dbGetFirst("SELECT user_id,imei,serial_number FROM container WHERE imei = ?", [imei]).then(res => {
            if (res) {
                for (let prob in tcpClients) {
                    if (tcpClients[prob].imei.toString() === imei.toString()) {
                        console.log("Same imei already connected! Disconnecting old one! IMEI => " + imei);
                        onDisconnected(tcpClients[prob]);
                    }
                }
                socket.write('OK:1\n');
                socket.imei = imei;
                socket.isAlive = true;
                socket.user = res.user_id;
                socket.name = res.serial_number;
                console.log("DEVICE CONNECTED! USER => " + socket.user + " NAME => " + socket.name);
                tcpClients[socket.id] = socket;
                sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
                    [socket.user, "IMEI", imei, socket.remoteAddress]);
                sendAllWsClients(socket.user,"CONNECTED")
            } else {
                console.log(socket.id + " - IMEI NOT FOUND! IMEI => " + imei);
                onDisconnected(socket);
            }
        });
        return;
    } else {
        sql.dbi("UPDATE container SET last_contact = CURRENT_TIMESTAMP WHERE user_id = ?", [socket.user]);
        let messages = message.split("|");
        messages.forEach(msg => {
            let paramIndex = msg.indexOf(":");
            let param = msg.slice(0, paramIndex).trim();
            let value = msg.slice(paramIndex + 1).trim();
            switch (param) {
                case 'IMEI':
                    console.log("IMEI IS ALREADY SET! IMEI => " + value + " USER => " + socket.user);
                    if (value === socket.imei) {
                        socket.write('OK:1\n');
                    } else {
                        console.log("IMEI IS NOT SAME! IMEI => " + value + " USER => " + socket.user);
                        onDisconnected(socket);
                    }
                    break;
                case 'K':
                    socket.isAlive = true;
                    break;
                case 'PRES DURUMU':
                case 'Pres':
                    if (value === 'Devre disi') {
                        sql.dbi("UPDATE container SET is_full = 1 WHERE user_id = ?", [socket.user]);
                    } else if (value === 'Yapildi') {
                        sql.dbi("UPDATE container SET total_press = total_press+1 WHERE user_id = ?", [socket.user]);
                    } else {
                        sql.dbi("UPDATE container SET is_full = 0 WHERE user_id = ?", [socket.user]);
                    }
                    break;
                case 'Manyetik Kilit':
                case 'Akım':
                case 'Yerel Zaman':
                case 'Çalışma Zamanı':
                    break;
                case 'KONUM':
                case 'Konum':
                    // KONUM:40.123456,29.123456
                    let gpsArray = value.split(",");
                    let lat = gpsArray[0];
                    let lng = gpsArray[1];
                    if (lat != 0.0 || lng != 0.0) {
                        sql.dbi("UPDATE address SET address_lat = ?,address_lng=? WHERE address_user_id = ?", [lat, lng, socket.user]);
                    } else {
                        console.log("GPS IS NULL! GPS => " + value + " USER => " + socket.user);
                    }
                    break;
                case 'Enlem':
                    // ENLEM:40.123456
                    if (value != 0.0) {
                        sql.dbi("UPDATE address SET address_lat = ? WHERE address_user_id = ?", [value, socket.user]);
                    } else {
                        console.log("LAT IS NULL! LAT => " + value + " USER => " + socket.user);
                    }
                    break;
                case 'Boylam':
                    // BOYLAM:29.123456
                    if (value != 0.0) {
                        sql.dbi("UPDATE address SET address_lng = ? WHERE address_user_id = ?", [value, socket.user]);
                    } else {
                        console.log("LNG IS NULL! LNG => " + value + " USER => " + socket.user);
                    }
                    break;
                case 'SICAKLIK':
                case 'Sıcaklık':
                    // SICAKLIK:21.50C
                    let temp = value.split("C")[0].trim();
                    sql.dbi("UPDATE container SET current_temperature = ? WHERE user_id = ?", [temp, socket.user]);
                    break;
                case 'KAPI':
                case 'Kapı':
                    // KAPAK:Acik
                    if (value === 'Acik') {
                        sql.db("SELECT is_open FROM container WHERE user_id = ?", [socket.user])
                        .then(res => {
                            if (res.is_open !== 1) {
                                sql.dbi("UPDATE container SET is_open = 1,last_opened=CURRENT_TIMESTAMP WHERE user_id = ?", [socket.user]);
                            }
                        });
                    } else {
                        sql.dbi("UPDATE container SET is_open = 0 WHERE user_id = ?", [socket.user]);
                    }
                    break;
                case 'KAPAK':
                case 'Kapak':
                    // KAPAK:Acik
                    if (value === 'Acik') {
                        sql.dbi("UPDATE container SET is_lid_open = 1 WHERE user_id = ?", [socket.user]);
                        setTimeout(() => {
                            console.log("LID IS STILL OPEN FOR CLIENT = " + socket.user);
                        }, 15000);
                    } else {
                        sql.dbi("UPDATE container SET is_lid_open = 0 WHERE user_id = ?", [socket.user]);
                    }
                    break;
                case 'DOLULUK':
                case 'Doluluk':
                    // DOLULUK: 50%
                    let fullness = value.split("%")[0].trim();
                    sql.dbi("UPDATE container SET current_fullness = ? WHERE user_id = ?",
                        [fullness, socket.user]);
                    break;
                case 'VOLTAJ':
                case 'Voltaj':
                    // VOLTAJ: 24.52
                    sql.dbi("UPDATE container SET current_voltage = ? WHERE user_id = ?", [value, socket.user]);
                    break;
                default:
                    console.log("UNKNOWN PARAMETER! PARAMETER => " + param + " VALUE => " + value + " USER => " + socket.user);
                    break;
            }
            sql.dbi("INSERT INTO container_log (user_id, param, value, ip) VALUES (?,?,?,?)",
                        [socket.user, param, value, socket.remoteAddress]);
        });
    }
    // socket.write('SEND OK\n');
    sendAllWsClients(socket.user,message)
    // console.log("FROM " + socket.user + " RECEIVED " + message);
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
        console.log('Client closed connection: ' + socket.user);
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