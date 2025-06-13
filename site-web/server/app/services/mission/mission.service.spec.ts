// import { expect } from 'chai';
// import * as sinon from 'sinon';
// import { MissionService } from '@app/services/mission/mission.service';
// import { Mission } from '@app/classes/mission/mission';
// import { Timer } from '@app/classes/timer/timer';
// import { ws_Methods } from '@common/events';
// import { MissionStatus } from '@common/action.robot';
// import { Socket } from 'socket.io';
// import { EventEmitter } from 'events';

// describe('MissionService', () => {
//   let service: MissionService;
//   let fakeSocket: Socket;

//   function createFakeSocket(): Socket {
//     return Object.assign(new EventEmitter(), {
//       emit: sinon.spy(),
//       id: 'fake-id',
//       nsp: '/',
//       client: {},
//       handshake: {},
//       connected: true,
//       disconnected: false,
//       rooms: new Set(),
//       data: {}
//     }) as unknown as Socket;
//   }

//   beforeEach(() => {
//     service = new MissionService();
//     fakeSocket = createFakeSocket();
//   });

//   describe('startMission', () => {
//     it('should create and start a mission', () => {
//       service.startMission('192.168.1.10', fakeSocket);
//       const mission = service.getMission('192.168.1.10');
//       expect(mission).to.be.instanceOf(Mission);
//       expect(service['sockets'].get('192.168.1.10')).to.equal(fakeSocket);
//       expect(mission?.active).to.be.true;
//     });

//     it('should reuse existing mission', () => {
//       service.startMission('192.168.1.10', fakeSocket);
//       const first = service.getMission('192.168.1.10');
//       service.startMission('192.168.1.10', fakeSocket);
//       const second = service.getMission('192.168.1.10');
//       expect(first).to.equal(second);
//     });
//   });

//   describe('stopMission', () => {
//     it('should stop and delete a mission', () => {
//       service.startMission('ip', fakeSocket);
//       const mission = service.getMission('ip');
//       const stopSpy = sinon.spy(mission!, 'stop');

//       service.stopMission('ip');
//       expect(service.getMission('ip')).to.be.undefined;
//       expect(service['sockets'].get('ip')).to.be.undefined;
//       expect(stopSpy.calledOnce).to.be.true;
//     });

//     it('should do nothing if mission does not exist', () => {
//       expect(() => service.stopMission('unknown')).to.not.throw();
//     });
//   });

//   describe('getMissionTimer', () => {
//     it('should return the timer of an existing mission', () => {
//       service.startMission('ip', fakeSocket);
//       const timer = service.getMissionTimer('ip');
//       expect(timer).to.be.instanceOf(Timer);
//     });

//     it('should return undefined if mission does not exist', () => {
//       expect(service.getMissionTimer('unknown')).to.be.undefined;
//     });
//   });

//   describe('listenRosSocket', () => {
//     it('should forward WebSocket message to RosMessagesService (string)', () => {
//       const rosSocket: any = {};
//       const socket = createFakeSocket();
//       const handleMessageStub = sinon.stub(service['rosMessageService'], 'handleMessage');

//       service.listenRosSocket('ip', socket, rosSocket, 'm1');
//       rosSocket.onmessage({ data: 'hello' });

//       expect(handleMessageStub.calledWith('ip', socket, { data: 'hello' }, 'm1')).to.be.true;
//     });

//     it('should handle buffer messages correctly', () => {
//       const rosSocket: any = {};
//       const socket = createFakeSocket();
//       const buffer = Buffer.from('from-buffer');
//       const handleMessageStub = sinon.stub(service['rosMessageService'], 'handleMessage');

//       service.listenRosSocket('ip', socket, rosSocket, 'm2');
//       rosSocket.onmessage({ data: buffer });

//       expect(handleMessageStub.calledWithMatch('ip', socket, { data: 'from-buffer' }, 'm2')).to.be.true;
//     });
//   });

//   describe('updateMissionStatus', () => {
//     it('should update mission status and emit to socket', () => {
//       service.startMission('ip', fakeSocket);
//       service.updateMissionStatus('ip', MissionStatus.RETURNING);
//       const mission = service.getMission('ip');
//       expect(mission?.status).to.equal(MissionStatus.RETURNING);
//       expect((fakeSocket.emit as sinon.SinonSpy).calledWith(ws_Methods.MISSION_STATUS_UPDATE, { status: MissionStatus.RETURNING })).to.be.true;
//     });

//     it('should log warning if mission or socket is missing', () => {
//       const consoleSpy = sinon.spy(console, 'warn');
//       service.updateMissionStatus('unknown-ip', MissionStatus.IDLE);
//       expect(consoleSpy.calledOnce).to.be.true;
//       consoleSpy.restore();
//     });
//   });
// });
import { expect } from 'chai';
import * as sinon from 'sinon';
import { MissionService } from '@app/services/mission/mission.service';
import { Mission } from '@app/classes/mission/mission';
import { Timer } from '@app/classes/timer/timer';
import { ws_Methods, ModeMission } from '@common/events';
import { MissionStatus } from '@common/action.robot';
import { Socket } from 'socket.io';
import { EventEmitter } from 'events';

describe('MissionService', () => {
  let service: MissionService;
  let fakeSocket: Socket;
  const fakeMode = ModeMission.PHYSICAL_ROBOT; // Choisis un mode par défaut ici

  function createFakeSocket(): Socket {
    return Object.assign(new EventEmitter(), {
      emit: sinon.spy(),
      id: 'fake-id',
      nsp: '/',
      client: {},
      handshake: {},
      connected: true,
      disconnected: false,
      rooms: new Set(),
      data: {}
    }) as unknown as Socket;
  }

  beforeEach(() => {
    service = new MissionService();
    fakeSocket = createFakeSocket();
  });

  describe('startMission', () => {
    it('should create and start a mission', () => {
      service.startMission('192.168.1.10', fakeSocket, fakeMode);
      const mission = service.getMission('192.168.1.10');
      expect(mission).to.be.instanceOf(Mission);
      expect(service['sockets'].get('192.168.1.10')).to.equal(fakeSocket);
      expect(mission?.active).to.be.true;
    });

    it('should reuse existing mission', () => {
      service.startMission('192.168.1.10', fakeSocket, fakeMode);
      const first = service.getMission('192.168.1.10');
      service.startMission('192.168.1.10', fakeSocket, fakeMode);
      const second = service.getMission('192.168.1.10');
      expect(first).to.equal(second);
    });
  });

  describe('stopMission', () => {
    it('should stop and delete a mission', () => {
      service.startMission('ip', fakeSocket, fakeMode);
      const mission = service.getMission('ip');
      const stopSpy = sinon.spy(mission!, 'stop');

      service.stopMission('ip');
      expect(service.getMission('ip')).to.be.undefined;
      expect(service['sockets'].get('ip')).to.be.undefined;
      expect(stopSpy.calledOnce).to.be.true;
    });

    it('should do nothing if mission does not exist', () => {
      expect(() => service.stopMission('unknown')).to.not.throw();
    });
  });

  describe('getMissionTimer', () => {
    it('should return the timer of an existing mission', () => {
      service.startMission('ip', fakeSocket, fakeMode);
      const timer = service.getMissionTimer('ip');
      expect(timer).to.be.instanceOf(Timer);
    });

    it('should return undefined if mission does not exist', () => {
      expect(service.getMissionTimer('unknown')).to.be.undefined;
    });
  });

  describe('listenRosSocket', () => {
    it('should forward WebSocket message to RosMessagesService (string)', () => {
      const rosSocket: any = {};
      const socket = createFakeSocket();
      const handleMessageStub = sinon.stub(service['rosMessageService'], 'handleMessage');

      service.listenRosSocket('ip', socket, rosSocket, 'm1');
      rosSocket.onmessage({ data: 'hello' });

      expect(handleMessageStub.calledWith('ip', socket, { data: 'hello' }, 'm1')).to.be.true;
    });

    it('should handle buffer messages correctly', () => {
      const rosSocket: any = {};
      const socket = createFakeSocket();
      const buffer = Buffer.from('from-buffer');
      const handleMessageStub = sinon.stub(service['rosMessageService'], 'handleMessage');

      service.listenRosSocket('ip', socket, rosSocket, 'm2');
      rosSocket.onmessage({ data: buffer });

      expect(handleMessageStub.calledWithMatch('ip', socket, { data: 'from-buffer' }, 'm2')).to.be.true;
    });
  });

  describe('updateMissionStatus', () => {
    it('should update mission status and emit to socket', () => {
      service.startMission('ip', fakeSocket, fakeMode);
      service.updateMissionStatus('ip', MissionStatus.RETURNING);
      const mission = service.getMission('ip');
      expect(mission?.status).to.equal(MissionStatus.RETURNING);
      expect((fakeSocket.emit as sinon.SinonSpy).calledWith(ws_Methods.MISSION_STATUS_UPDATE, { status: MissionStatus.RETURNING })).to.be.true;
    });

    it('should log warning if mission or socket is missing', () => {
      const consoleSpy = sinon.spy(console, 'warn');
      service.updateMissionStatus('unknown-ip', MissionStatus.IDLE);
      expect(consoleSpy.calledOnce).to.be.true;
      consoleSpy.restore();
    });
  });
});
