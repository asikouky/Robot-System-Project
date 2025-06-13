import { Application } from '@app/app';
import { Server } from '@app/server';
import { SocketManager } from '@app/services/socket/socket-manager/socketManager.service';
import { expect } from 'chai';
import * as sinon from 'sinon';
import { io as ioClient, Socket } from 'socket.io-client';
import { Container } from 'typedi';

const RESPONSE_DELAY = 200;

describe('SocketManager service tests', () => {
    let service: SocketManager;
    let server: Server;
    let clientSocket: Socket;

    const urlString = 'http://localhost:3000/';

    beforeEach(async () => {
        const app = Container.get(Application);
        server = new Server(app);
        server.init();
        service = server['socketManager'];
        sinon.stub(console, 'log');
    });

    afterEach(() => {
        clientSocket.close();
        service['sio'].close();
        sinon.restore();
    });

    it('should call handleSocket from the services', (done) => {
        const missionRoomHandleSocketsStub = sinon.stub(service.missionRoomService, 'handleSockets');

        clientSocket = ioClient(urlString);

        setTimeout(() => {
            expect(missionRoomHandleSocketsStub.called).to.equal(true);
            done();
        }, RESPONSE_DELAY);
    });
});
