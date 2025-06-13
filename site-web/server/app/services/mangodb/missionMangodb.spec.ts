import { MongoMemoryServer } from 'mongodb-memory-server';
import { MongoClient, Db } from 'mongodb';
import { MissionDocument } from './missionMangodb';
import { MissionStatus } from '@common/action.robot';
import { expect } from 'chai';

describe('MongoDB Mission Tests', () => {
  let mongoServer: MongoMemoryServer;
  let uri: string;
  let client: MongoClient;
  let db: Db;

  before(async function () {
    this.timeout(30000); // allonge le délai pour démarrer Mongo

    mongoServer = await MongoMemoryServer.create({
      binary: {
        version: '6.0.5', // ✅ version compatible AVX-free
      },
    });

    uri = mongoServer.getUri();
    client = new MongoClient(uri);
    await client.connect();
    db = client.db('inf3995');
  });

  after(async () => {
    if (client) await client.close();
    if (mongoServer) await mongoServer.stop();
  });

  it('devrait insérer une mission et la retrouver', async () => {
    const missions = db.collection<MissionDocument>('missions');

    const mission: MissionDocument = {
      missionId: 'mission-test-001',
      ip: '192.168.1.100',
      status: MissionStatus.IN_MISSION,
      createdAt: new Date(),
      mode: 'SIMULATION',
      duration: 180,
      distanceTravelled: { '192.168.1.100': 12.5 },
      mapImage: null,
      logFilename: 'mission-001.log',
    };

    const insertResult = await missions.insertOne(mission);
    expect(insertResult.insertedId).to.not.be.undefined;

    const fetched = await missions.findOne({ _id: insertResult.insertedId });
    expect(fetched?.missionId).to.equal('mission-test-001');
    expect(fetched?.distanceTravelled['192.168.1.100']).to.equal(12.5);
  });

  it('devrait supprimer une mission', async () => {
    const missions = db.collection<MissionDocument>('missions');

    const inserted = await missions.insertOne({
      missionId: 'to-delete',
      ip: '192.168.1.101',
      status: MissionStatus.IDLE,
      createdAt: new Date(),
      mode: 'PHYSIQUE',
      duration: 90,
      distanceTravelled: { '192.168.1.101': 6.2 },
    });

    const deleteResult = await missions.deleteOne({ _id: inserted.insertedId });
    expect(deleteResult.deletedCount).to.equal(1);

    const result = await missions.findOne({ _id: inserted.insertedId });
    expect(result).to.be.null;
  });
});
