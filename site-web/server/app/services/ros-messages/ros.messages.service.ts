import { MissionService } from "@app/services/mission/mission.service";
import { EventType, ws_Methods } from "@common/events";
import { Socket } from "socket.io";

export class RosMessagesService {
    constructor(
        private missionService: MissionService
    ) {}

    handleMessage(ip: string, socket: Socket, messageIn: {data: any}, missionId: string): void {
        try {
            const message = JSON.parse(messageIn.data);

            switch (message.eventType) {
                case EventType.MISSION_STATUS:
                    this.handleMissionStatus(ip, message.data);
                    break;
                case EventType.LOGS:
                    this.handleDebugLaserScanMessage(socket, message, ip, missionId);
                    break;
                case EventType.BATTERY_LEVEL:
                    this.handleBatteryMessage(socket, message.data);
                    break;
                case EventType.OBJECT_DETECTION:
                    this.handleObjectDetection(message.data);
                    break;
                case EventType.CONNECTION_STATUS:
                    this.handleConnectionStatus(message.data);
                    break;
                case EventType.IDENTIFICATION:
                    this.handleRobotIdentificationMessage(socket, message.data);
                    break;
                case EventType.MAP:
                    this.handleMapMessage(ip, message);
                    break;
                default:
                    console.warn(`[ROS Message Handler] ⚠️ Type d'événement inconnu: ${message.eventType}`);
            }
        } catch (error) {
            console.error("[ROS Message Handler] ❌ Erreur lors du parsing du message :", error);
        }
  
    }

    private handleMissionStatus(ip: string, data: any) {
        console.log(`[ROS Message Handler] Mise à jour du statut de mission :`, data);
        const status = typeof data === 'string' ? data : data.status;

        if (!status) {
            console.warn("[ROS Message Handler] Données de mission incomplètes :", data);
            return;
        }

        this.missionService.updateMissionStatus(ip, status);
    }

    private handleObjectDetection(data: any) {
        console.log(`[ROS Message Handler] 🧠 Détection d'objet :`, data);
    }

    private handleConnectionStatus(data: any) {
        console.log(`[ROS Message Handler] 🌐 État de connexion mis à jour :`, data);
    }

    // old version
    private handleBatteryMessage(socket: Socket, data: any) {
        if (data.battery !== undefined) {  
            console.log(`[WebSocket] 🔋 Niveau de batterie reçu : ${data.battery}%`);
            socket.emit('battery_update', { battery: data.battery });
        }
    }

    private handleRobotIdentificationMessage(socket: Socket, data: any) {
        if (data.status === 'identification started') {
            console.log(`[WebSocket] Identification reçu par le robot.`);
            socket.emit(ws_Methods.STATUS, 'Identification en cours');
        } else if (data.status === 'completed') {
            console.log(`[WebSocket] Identification terminée pour le robot.`);
            socket.emit(ws_Methods.MISSION_AVAILABILITY, true);
            socket.emit(ws_Methods.STATUS, 'Identification terminée');
        } else {
            console.warn(`[WebSocket] Type de message inconnu reçu de ROS :`, data);
        }
    }

    private handleMapMessage(ip: string, data: any){
        if (data.type === 'map_image') {
            this.missionService.updateMapImage(ip, data.data.image)
        }
    }

    private handleDebugLaserScanMessage(socket: Socket, data: any, ip?: string, missionId?: string) {
        const laserData = data.data;
        const robot = data.robot;
        const logString = `${robot} : SCAN_LOG: t=${laserData.timestamp} | ` +
            `min_dist=${laserData.min_distance}m@${laserData.min_angle_deg}° | ` +
            `obstacles(<0.5m)=${laserData.close_obstacles} | ` +
            `F/L/R/B=${laserData.front_min}/${laserData.left_min}/${laserData.right_min}/${laserData.back_min}m | ` +
            `avg=${laserData.average_distance}m var=${laserData.distance_variance}\n`;

        socket.emit(ws_Methods.LASER_SCAN_LOG, logString);
        
        this.missionService.writeToMissionLog(ip, logString);
    }
}
