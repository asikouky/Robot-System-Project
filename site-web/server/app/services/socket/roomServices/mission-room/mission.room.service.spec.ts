import { ClientAccessService } from '@app/services/client-access/client.access.service';
import { MissionRoomService } from '@app/services/socket/roomServices/mission-room/mission.room.service';
import { ws_Methods } from '@common/events';
import { expect } from 'chai';
import { createServer } from 'http';
import * as sinon from 'sinon';
import { Server, Socket } from 'socket.io';
import { WebSocket } from 'ws';

describe('MissionRoomService', () => {
    let missionRoomService: MissionRoomService;
    let clientAccessServiceStub: sinon.SinonStubbedInstance<ClientAccessService>;
    let ioServerMock: Server;
    let socketMock: Socket;
    let rosSocketMock: sinon.SinonStubbedInstance<WebSocket>;
    let sandbox: sinon.SinonSandbox;

    beforeEach(async () => {
        sandbox = sinon.createSandbox();

        
        const httpServer = createServer();
        ioServerMock = new Server(httpServer, { cors: { origin: '*', methods: ['GET', 'POST'] } });

        
        clientAccessServiceStub = sandbox.createStubInstance(ClientAccessService);

        
        missionRoomService = new MissionRoomService(ioServerMock);

        
        await new Promise<void>((resolve) => {
            ioServerMock.on('connection', (socket) => {
                socketMock = socket;
                resolve(); 
            });

            
            ioServerMock.emit('connection', ioServerMock.of('/').sockets);
        });

        
        rosSocketMock = sandbox.createStubInstance(WebSocket);

        
        sandbox.stub(console, 'log');
        (missionRoomService as any).clientAccessService = clientAccessServiceStub;
        (missionRoomService as any).missionService = {
            getMissionId: sinon.stub().returns('mission123'),
            listenRosSocket: sinon.stub(),
            startMission: sinon.stub(),
            stopMission: sinon.stub(),
            writeToMissionLog: sinon.stub(),
            getMissionTimer: sinon.stub().returns({
                on: sinon.stub(),
            }),
        };
    });

    afterEach(() => {
        sandbox.restore();
        ioServerMock.close(); 
    });

    it(' devrait instancier le service avec une référence à io.Server', () => {
        expect(missionRoomService).to.be.instanceOf(MissionRoomService);
    });

    describe('handleSockets()', () => {
        beforeEach(() => {
            sandbox.stub(socketMock, 'on').callsFake((event, callback) => {
                if (event === ws_Methods.IDENTIFY) {
                    callback({ ip: '192.168.1.100' });
                }
                return socketMock;
            });
        });

        it(' devrait gérer les connexions et vérifier les IPs', () => {
            sandbox.stub(clientAccessServiceStub, 'isClientConnected').returns(false);
            sandbox.stub(clientAccessServiceStub, 'isIpRegistered').returns(false);

            missionRoomService.handleSockets(socketMock);

            expect(clientAccessServiceStub.isClientConnected.calledWith('192.168.1.100', socketMock.id)).to.be.true;
            expect(clientAccessServiceStub.isIpRegistered.calledWith('192.168.1.100')).to.be.true;
        });

        it(" devrait refuser une connexion si l'IP est déjà attribuée", () => {
            sandbox.stub(clientAccessServiceStub, 'isClientConnected').returns(false);
            sandbox.stub(clientAccessServiceStub, 'isIpRegistered').returns(true);

            const emitSpy = sandbox.spy(socketMock, 'emit');

            missionRoomService.handleSockets(socketMock);

            expect(emitSpy.calledWith(ws_Methods.ERROR, `L'adresse IP 192.168.1.100 est déjà en cours d'utilisation.`)).to.be.true;
        });

        it(" devrait ouvrir un WebSocket ROS et ajouter l'IP à la liste", () => {
            sandbox.stub(rosSocketMock, 'onopen').value(() => {
                missionRoomService['rosSockets'].set('192.168.1.100', rosSocketMock);
            });

            missionRoomService.handleSockets(socketMock);

            expect(missionRoomService['rosSockets'].has('192.168.1.100')).to.be.true;
        });
    });

    describe('LAUNCH_MISSION et STOP_MISSION', () => {
        beforeEach(() => {
            missionRoomService['rosSockets'].set('192.168.1.100', rosSocketMock);
        });

        it(' devrait envoyer une commande "identify" au robot lors du lancement de mission', () => {
            const sendSpy = sandbox.spy(rosSocketMock, 'send');

            sandbox.stub(socketMock, 'on').callsFake((event, callback) => {
                if (event === ws_Methods.LAUNCH_MISSION) {
                    callback({ ip: '192.168.1.100' });
                }
                return socketMock;
            });

            missionRoomService.handleSockets(socketMock);

            expect(sendSpy.calledWith(JSON.stringify({ command: 'identify' }))).to.be.true;
        });

        it(' devrait envoyer une commande "identify" au robot lors de l\'arrêt de mission', () => {
            const sendSpy = sandbox.spy(rosSocketMock, 'send');

            sandbox.stub(socketMock, 'on').callsFake((event, callback) => {
                if (event === ws_Methods.STOP_MISSION) {
                    callback({ ip: '192.168.1.100' });
                }
                return socketMock;
            });

            missionRoomService.handleSockets(socketMock);

            expect(sendSpy.calledWith(JSON.stringify({ command: 'identify' }))).to.be.true;
        });
    });

    describe('disconnect()', () => {
        it("devrait libérer l'IP du client et fermer le WebSocket ROS", () => {
            missionRoomService['rosSockets'].set('192.168.1.100', rosSocketMock);

            sandbox.stub(socketMock, 'on').callsFake((event, callback) => {
                if (event === 'disconnect') {
                    callback();
                }
                return socketMock;
            });

            missionRoomService.handleSockets(socketMock);

            expect(missionRoomService['rosSockets'].has('192.168.1.100')).to.be.false;
        });
    });

    describe('releaseIP()', () => {
        it(" devrait notifier que l'IP est libérée", () => {
            sandbox.stub(clientAccessServiceStub, 'removeClientBySocket').returns('192.168.1.100');
            const emitSpy = sandbox.spy(ioServerMock, 'emit');

            missionRoomService['rosSockets'].set('192.168.1.100', rosSocketMock);
            missionRoomService['releaseIP'](socketMock.id);

            expect(emitSpy.calledWith(ws_Methods.STATUS, `L'adresse IP (192.168.1.100) vient d'être libérée.`)).to.be.true;
        });
    });
});
