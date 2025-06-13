import { MongoClient, Db } from 'mongodb';
import { ObjectId } from 'mongodb';
import { MissionStatus } from '@common/action.robot';
//import { Timer } app/clasfrom "@ses/timer/timer";
import { ModeMission } from '@common/events';
export interface MissionDocument {
  _id?: ObjectId;
  missionId: string;
  ip: string;
  ip2?: string;
  mode: ModeMission;
  status: MissionStatus;
  createdAt: Date;
  duration: number; // en secondes
  distanceTravelled: {
    [ip: string]: number; 
  };
  mapImage?: string | null;
  logFilename?: string;
  logHistory?: string[];
}

const uri =  'mongodb://localhost:27017' //'mongodb://localhost:27017';
const dbName = 'inf3995';
let dbInstance: Db | null = null;

export async function connectToDatabase() {
  if (dbInstance) return dbInstance;

  try {
    const client = new MongoClient(uri);
    await client.connect();
    console.log('✅ Connecté à MongoDB');

    dbInstance = client.db(dbName);
    return dbInstance;
  } catch (error) {
    console.error('❌ Erreur lors de la connexion à MongoDB :', error);
    throw error;
  }
  
}
