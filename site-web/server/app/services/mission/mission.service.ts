import { Mission } from "@app/classes/mission/mission";
import { Timer } from "@app/classes/timer/timer";
import { RosMessagesService } from "@app/services/ros-messages/ros.messages.service";
import { MissionStatus } from "@common/action.robot";
import { ModeMission, ws_Methods } from "@common/events";
import { EventEmitter } from "events";
import * as fs from 'fs';
import * as path from 'path';
import { Socket } from "socket.io";
import { Service } from "typedi";
import { WebSocket } from "ws";
import { connectToDatabase } from "@app/services/mangodb/missionMangodb";


@Service()
export class MissionService extends EventEmitter {
    private missions: Map<string, Mission> = new Map();
    private sockets: Map<string, Socket> = new Map(); // IP → Socket
    private rosMessageService: RosMessagesService;
    
    
    constructor() {
        super();
        this.rosMessageService = new RosMessagesService(this);
    }

    private normalizeIp(ip: string): string {
        return ip.trim(); // Ajoute .toLowerCase() ici si nécessaire
    }
   
    startMission(ip: string, socket: Socket, mode: ModeMission) {
        const normIp = this.normalizeIp(ip);

        if (!this.missions.has(normIp)) {
            this.missions.set(normIp, new Mission(normIp, mode));
        }

        this.sockets.set(normIp, socket);

        const mission = this.missions.get(normIp)!;
        
        mission.start();
       
        
    }

    stopMission(ip: string) {
        const normIp = this.normalizeIp(ip);

        const mission = this.missions.get(normIp);
        if (!mission) return;

        mission.stop();
       
    }

    getMission(ip: string): Mission | undefined {
        return this.missions.get(this.normalizeIp(ip));
    }

    getMissionId(ip: string): string | undefined {
        const mission = this.missions.get(this.normalizeIp(ip));
        return mission ? mission.missionId : undefined;
    }
    
    getMissionTimer(ip: string): Timer | undefined {
        return this.getMission(ip)?.timer;
    }

    listenRosSocket(ip: string, socket: Socket, rosSocket: WebSocket, missionId: string) {
        rosSocket.onmessage = (message) => {
            const raw =
                typeof message.data === 'string'
                    ? message.data
                    : Buffer.isBuffer(message.data)
                    ? message.data.toString()
                    : '';
            this.rosMessageService.handleMessage(ip, socket, { data: raw }, missionId);
        };
    }

    updateMissionStatus(ip: string, status: MissionStatus) {
        const normIp = this.normalizeIp(ip);

        const mission = this.missions.get(normIp);
        const socket = this.sockets.get(normIp);

        if (!mission || !socket) {
            console.warn(`[MissionService] Impossible de mettre à jour le statut. Mission ou socket manquant pour ${normIp}`);
            return;
        }

        mission.status = status;
        socket.emit(ws_Methods.MISSION_STATUS_UPDATE, { status });
    }

    updateMapImage(ip: string, mapBase64: string) {
        const normIp = this.normalizeIp(ip);

        const mission = this.missions.get(normIp);
        const socket = this.sockets.get(normIp);

        if (!mission || !socket) {
            console.log(mission, socket);
            console.warn(`[MissionService] Impossible de mettre à jour la map. Mission ou socket manquant pour ${normIp}`);
            return;
        }

        mission.mapImage = mapBase64;

        socket.emit(ws_Methods.MAP_IMAGE_UPDATE, {
            mapImage: mapBase64,
        });
    }

    addSecondaryRobot(ipPrimary: string, ipSecondary: string, socket: Socket) {
        const normPrimary = this.normalizeIp(ipPrimary);
        const normSecondary = this.normalizeIp(ipSecondary);
    
        const mission = this.missions.get(normPrimary);
        if (!mission) {
            console.warn(`[MissionService] Aucune mission trouvée pour IP primaire ${normPrimary}`);
            return;
        }
    
        mission.setSecondaryIp(normSecondary);
        this.sockets.set(normSecondary, socket);
    }
    

   async writeToMissionLog(ip: string, logMessage: string): Promise<void> {
        const missionId = this.getMissionId(ip);
        
        if (!missionId) {
            return;
        }
        
        const findRootDir = () => {
            let currentDir = __dirname;
            while (!fs.existsSync(path.join(currentDir, 'package.json'))) {
                const parentDir = path.dirname(currentDir);
                if (parentDir === currentDir) {
                    return __dirname; 
                }
                currentDir = parentDir;
            }
            return currentDir; 
        };
    
        const serverRoot = findRootDir();
        const logDir = path.join(serverRoot, 'logs');
        
        if (!fs.existsSync(logDir)) {
            fs.mkdirSync(logDir, { recursive: true });
        }
        
        const fileName = `${missionId}.log`;
        const filePath = path.join(logDir, fileName);
    
        fs.appendFile(
            filePath,
            logMessage,
            (err) => {
                if (err) {
                    console.error(`[WebSocket] Erreur en ajoutant au fichier de logs: ${err.message}`);
                }
            }
        );
        try {
            try {
                const db = await connectToDatabase();
                await db.collection<{ missionId: string; logHistory: string[] }>('missions').updateOne(
                    { missionId }, 
                    { $push: { logHistory: logMessage } }
                );
                console.log(`🗃️ Log aussi enregistré dans MongoDB pour ${missionId}`);
            } catch (mongoErr) {
                console.error(`❌ Erreur ajout log MongoDB: ${mongoErr.message}`);
            }
            console.log(`🗃️ Log aussi enregistré dans MongoDB pour ${missionId}`);
          } catch (mongoErr) {
            console.error(`❌ Erreur ajout log MongoDB: ${mongoErr.message}`);
          }
    }
    
}
