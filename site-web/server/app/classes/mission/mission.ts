import { Timer } from "@app/classes/timer/timer";
import { MissionStatus } from "@common/action.robot";
import { connectToDatabase } from "@app/services/mangodb/missionMangodb";
import { MissionDocument } from "@app/services/mangodb/missionMangodb";
import { ModeMission } from "@common/events";

export class Mission {
    missionId: string;
    ipPrimary: string;
    ipSecondary?: string;
    mode: ModeMission;
    status: MissionStatus;
    timeLeft: number;
    timer: Timer;
    active: boolean;
    mapImage: string | null;
    logFilename: string;

    duration: number = 0;
    distancePrimary: number = 0;
    distanceSecondary?: number;
    
    constructor(ipPrimary: string, mode: ModeMission) {
        this.ipPrimary = ipPrimary;
        this.mode = mode;
        this.missionId = this.generateMissionId(ipPrimary);
        this.status = MissionStatus.IDLE;
        this.timeLeft = 0;
        this.timer = new Timer();
        this.active = false;
        this.mapImage = null;
        this.logFilename =`${this.missionId}.log`
    }
    private generateMissionId(ip: string): string {
        const now = new Date();
        const datePart = now.toISOString().replace(/[-:T.Z]/g, '').slice(0, 14);
        const randomPart = Math.random().toString(36).substring(2, 8);
        return `${ip.replace(/\./g, '_')}_${datePart}_${randomPart}`;
    }

    setSecondaryIp(ip: string) {
        this.ipSecondary = ip;
    }

    start() {
        this.status = MissionStatus.IN_MISSION;
        this.active = true;
        this.timer.startTimer(this.timeLeft);
        
        
        
    }

    async stop() {
        this.status = MissionStatus.IDLE;
        this.active = false;
        this.timer.stopTimer();
        this.duration = this.timer.getTime(); // avoir 
        
        await this.saveToDatabase();
        
       
    };
    async saveToDatabase() {
        try {
            const db = await connectToDatabase();
            const collection = db.collection<MissionDocument>('missions');
            
            const doc: MissionDocument = {
                missionId: this.missionId,
                ip: this.ipPrimary,
                ip2: this.ipSecondary,
                status: this.status,
                mode: this.mode,
                createdAt: new Date(),
                duration: this.timer.getTime(), 
                distanceTravelled: {
                  [this.ipPrimary]: this.distancePrimary ?? 0,
                  ...(this.ipSecondary ? { [this.ipSecondary]: this.distanceSecondary ?? 0 } : {})
                },
                mapImage: this.mapImage ?? null,
                logFilename: this.logFilename
              };
              
    
            await collection.insertOne(doc);
            
            console.log('✅ Mission enregistrée dans la base de données', doc);

            

        } catch (error) {
            console.error('❌ Erreur lors de l’enregistrement de la mission', error);
        } 
        
    }
    
    
    
}   

