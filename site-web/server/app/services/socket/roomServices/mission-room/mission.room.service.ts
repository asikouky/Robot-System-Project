import { ClientAccessService } from '@app/services/client-access/client.access.service';
import { MissionService } from '@app/services/mission/mission.service';
import { ModeMission, ws_Methods } from '@common/events';
import * as io from 'socket.io';
import { Socket } from 'socket.io';
import { Service } from 'typedi';
import { WebSocket } from 'ws';

@Service()
export class MissionRoomService {
    private clientAccessService: ClientAccessService;
    private sio: io.Server;
    private rosSockets: Map<string, WebSocket> = new Map(); // Map <IP, WebSocket>
    private missionService: MissionService;

    constructor(sio: io.Server) {
        this.clientAccessService = new ClientAccessService();
        this.missionService = new MissionService();
        this.sio = sio;
    }

    handleSockets(socket: Socket) {
        socket.on(ws_Methods.IDENTIFY, (data: { ip: string, numRobot:string, mode: ModeMission}) => {
            if (!data.ip) {
                socket.emit(ws_Methods.ERROR, 'Aucune adresse IP fournie.');
                return;
            }

            if (!this.clientAccessService.isClientConnected(data.ip, socket.id) && this.clientAccessService.isIpRegistered(data.ip)) {
                socket.emit(ws_Methods.ERROR, `L'adresse IP ${data.ip} est déjà en cours d'utilisation.`);
                
                
            }

            console.log(`[WebSocket] Tentative de connexion à ROS WebSocket sur ${data.ip}:8765 ...`);

            const rosWebSocketUrl = `ws://${data.ip}:8765`;
            const rosSocket = new WebSocket(rosWebSocketUrl);

            const clientIp = data.ip || (data.ip ? data.ip : "unknown");
            const missionId = this.missionService.getMissionId(clientIp);
            this.missionService.listenRosSocket(data.ip, socket, rosSocket, missionId);
            console.log(data.numRobot);
            
            rosSocket.onopen = () => {
                console.log(`[WebSocket] Connecté à ROS WebSocket sur ${rosWebSocketUrl}`);
                socket.broadcast.emit(ws_Methods.STATUS, `L'adresse IP (${data.ip}) vient d'être utilisé.`);
                this.rosSockets.set(data.ip, rosSocket);

                // Assigner la room dynamique
                const roomName = `MISSION_ROOM_${data.ip}`;
                socket.join(roomName);
                console.log(`[MissionRoomService] Client ${socket.id} rejoint ${roomName}`);

                this.clientAccessService.registerClient(data.ip, socket.id);
                socket.emit(ws_Methods.STATUS, `Connecté à ROS WebSocket sur ${data.ip}`);

                console.log(data.numRobot);
                this.sendToRos(data.ip, { command: 'identify', target_ip: `${data.ip}`, topic_prefix: `robot${data.numRobot}-104`, robot: `limo${data.numRobot}` });
            };

            rosSocket.onerror = () => {
                console.error(`[WebSocket] Erreur de connexion à ROS sur ${data.ip}`);
                socket.emit(ws_Methods.ERROR, `Impossible de se connecter à ROS sur ${data.ip}`);
            };

            rosSocket.onclose = () => {
                console.log(`[WebSocket] Connexion fermée avec ROS (${data.ip}).`);
                socket.emit(ws_Methods.MISSION_AVAILABILITY, true);

                this.missionService.stopMission(data.ip);
                console.log(`[WebSocket] Arrêt de la mission ${missionId} pour le robot (${data.ip}).`);
                socket.emit(ws_Methods.STATUS, `Mission ${missionId} arrêtée`);
                
                // Log mission stop
                const missionStopMsg = `MISSION_LOG: t=${this.getTimestamp()} | Mission ${missionId} STOPPED for robot at ${data.ip}\n`;
                this.missionService.writeToMissionLog(data.ip, missionStopMsg);
                
                this.rosSockets.delete(data.ip);
                //this.missionIds.delete(data.ip);
                this.releaseIP(socket.id);
            };
        });


        socket.on(ws_Methods.LAUNCH_MISSION, (data: { ip: string, ip2: string, mode: ModeMission }) => {
            const rosSocket = this.rosSockets.get(data.ip);
            const rosSocket2 = data.ip2 ? this.rosSockets.get(data.ip2) : undefined;

            if (rosSocket && rosSocket.readyState === WebSocket.OPEN) {
                if (data.ip2) {
                    rosSocket.send(JSON.stringify({ command: 'peer_ip', data: { otherIp: data.ip2 } }));
                }
                console.log(data.mode);
                this.missionService.startMission(data.ip, socket, data.mode);
                
                this.startSendingTime(socket, data.ip);
                 
              
                
                const missionId = this.missionService.getMissionId(data.ip);
                console.log(`[WebSocket] Lancement de la mission ${missionId} pour le robot (${data.ip}).`);
                rosSocket.send(JSON.stringify({ command: 'start', missionId }));
                socket.emit(ws_Methods.STATUS, `Mission ${missionId} lancée`);
                
                // Log mission start
                const missionStartMsg = `MISSION_LOG: t=${this.getTimestamp()} | Mission ${missionId} STARTED for robot at ${data.ip}\n`;
                this.missionService.writeToMissionLog(data.ip, missionStartMsg);

                if (data.ip2 && rosSocket2 && rosSocket2.readyState === WebSocket.OPEN) {
                    //  Envoyer IP du robot principal au secondaire
                    rosSocket2.send(JSON.stringify({ command: 'peer_ip', data: { otherIp: data.ip } }));
                }

            } else {
                socket.emit(ws_Methods.ERROR, `Impossible de lancer la mission.`);
            }
        });

        socket.on(ws_Methods.STOP_MISSION, (data: { ip: string }) => {
            const rosSocket = this.rosSockets.get(data.ip);
            const missionId = this.missionService.getMissionId(data.ip) || 'unknown';

            if (rosSocket && rosSocket.readyState === WebSocket.OPEN) {
                this.missionService.stopMission(data.ip);
                console.log(`[WebSocket] Arrêt de la mission ${missionId} pour le robot (${data.ip}).`);
                
                rosSocket.send(JSON.stringify({ command: 'stop' }));
                socket.emit(ws_Methods.STATUS, `Mission ${missionId} arrêtée`);
                
                // Log mission stop
                const missionStopMsg = `MISSION_LOG: t=${this.getTimestamp()} | Mission ${missionId} STOPPED for robot at ${data.ip}\n`;
                this.missionService.writeToMissionLog(data.ip, missionStopMsg);
            } else {
                socket.emit(ws_Methods.ERROR, `Impossible d'arrêter la mission.`);
            }
        });

        socket.on(ws_Methods.GET_BATTERY, (data: { ip: string }) => {
            const rosSocket = this.rosSockets.get(data.ip);
        
            if (rosSocket && rosSocket.readyState === WebSocket.OPEN) {
                console.log(`[WebSocket] Demande de niveau de batterie envoyée au robot (${data.ip}).`);
                rosSocket.send(JSON.stringify({ command: 'battery_status' }));
            } else {
                socket.emit(ws_Methods.ERROR, `Impossible d'obtenir le niveau de batterie.`);
            }
        });
        

        socket.on('disconnect', () => {
            console.log(`[MissionRoomService] Client ${socket.id} déconnecté.`);
            this.releaseIP(socket.id);
        });
    }

    private startSendingTime(socket: Socket, ip: string) {
        this.missionService.getMissionTimer(ip).on("timeUpdated", (time: number) => {
            socket.emit(ws_Methods.TIMER_UPDATE, { time });
        });
    }

    private getTimestamp(): string {
        return new Date().toISOString();
    }

    private sendToRos(ip: string, data: object) {
        const rosSocket = this.rosSockets.get(ip);
        if (rosSocket && rosSocket.readyState === WebSocket.OPEN) {
            rosSocket.send(JSON.stringify(data));
        } else {
            console.error(`[WebSocket] ROS WebSocket non connecté pour IP ${ip}`);
        }
    }

    private releaseIP(clientId: string) {
        
        const releasedIP = this.clientAccessService.removeClientBySocket(clientId);
        if (releasedIP !== '') this.sio.emit(ws_Methods.STATUS, `L'adresse IP (${releasedIP}) vient d'être libéré.`);
    }

}