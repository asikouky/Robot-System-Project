// // import { expect } from 'chai';
// // import * as sinon from 'sinon';
// // import { RosMessagesService } from '@app/services/ros-messages/ros.messages.service';
// // import { MissionService } from '@app/services/mission/mission.service';
// // import { ws_Methods, EventType } from '@common/events';
// // import * as fs from 'fs';
// // import * as path from 'path';
// // import { Socket } from 'socket.io';

// // describe('RosMessagesService', () => {
// //     let service: RosMessagesService;
// //     let missionService: MissionService;
// //     let fakeSocket: Socket;
// //     let socketEmitSpy: sinon.SinonSpy;

// //     function createFakeSocket(): Socket {
// //         const socket = {
// //             emit: sinon.spy(),
// //         } as unknown as Socket;
// //         socketEmitSpy = socket.emit as sinon.SinonSpy;
// //         return socket;
// //     }

// //     beforeEach(() => {
// //         missionService = new MissionService();
// //         service = new RosMessagesService(missionService);
// //         fakeSocket = createFakeSocket();
// //     });

// //     describe('handleMessage', () => {
// //         it('should warn if mission status data is missing', () => {
// //             const warnSpy = sinon.spy(console, 'warn');
// //             const missionServiceSpy = sinon.spy(missionService, 'updateMissionStatus');

// //             const message = {
// //                 eventType: EventType.MISSION_STATUS,
// //                 data: {}, // no status
// //             };

// //             service.handleMessage('192.168.1.1', fakeSocket, { data: JSON.stringify(message) }, 'missionXYZ');

// //             expect(missionServiceSpy.notCalled).to.be.true;
// //             expect(warnSpy.calledWithMatch('[ROS Message Handler] Données de mission incomplètes')).to.be.true;

// //             warnSpy.restore();
// //         });
// //         it('should warn on unknown IDENTIFICATION status', () => {
// //             const warnSpy = sinon.spy(console, 'warn');

// //             const message = JSON.stringify({
// //                 eventType: EventType.IDENTIFICATION,
// //                 data: { status: 'unexpected' },
// //             });

// //             service.handleMessage('ip', fakeSocket, { data: message }, 'missionX');

// //             expect(warnSpy.calledWithMatch('[WebSocket] Type de message inconnu reçu de ROS :')).to.be.true;
// //             warnSpy.restore();
// //         });

// //         it('should handle LOGS event', () => {
// //             const data = {
// //                 eventType: EventType.LOGS,
// //                 data: {
// //                     timestamp: '1234',
// //                     min_distance: 1,
// //                     min_angle_deg: 45,
// //                     close_obstacles: 2,
// //                     front_min: 1,
// //                     left_min: 2,
// //                     right_min: 3,
// //                     back_min: 4,
// //                     average_distance: 2,
// //                     distance_variance: 0.5,
// //                 },
// //             };

// //             const jsonData = JSON.stringify(data);

// //             const appendStub = sinon.stub(fs, 'appendFile').callsFake((_path, _log, cb) => cb?.(null));
// //             const existsSyncStub = sinon.stub(fs, 'existsSync').returns(true);
// //             sinon.stub(path, 'join').callsFake((...args: string[]) => args.join('/'));

// //             service.handleMessage('ip', fakeSocket, { data: jsonData }, 'mission123');
// //             expect(socketEmitSpy.calledWith(ws_Methods.LASER_SCAN_LOG, sinon.match.string)).to.be.true;

// //             appendStub.restore();
// //             existsSyncStub.restore();
// //             (path.join as any).restore?.();
// //         });

// //         it('should handle BATTERY_LEVEL event', () => {
// //             const message = JSON.stringify({ eventType: EventType.BATTERY_LEVEL, data: { battery: 87 } });
// //             service.handleMessage('ip', fakeSocket, { data: message }, 'mission1');
// //             expect(socketEmitSpy.calledWith('battery_update', { battery: 87 })).to.be.true;
// //         });

// //         it('should handle OBJECT_DETECTION event', () => {
// //             const message = JSON.stringify({ eventType: EventType.OBJECT_DETECTION, data: { detected: true } });
// //             const logSpy = sinon.spy(console, 'log');
// //             service.handleMessage('ip', fakeSocket, { data: message }, 'mission1');
// //             expect(logSpy.calledWithMatch("[ROS Message Handler] 🧠 Détection d'objet")).to.be.true;
// //             logSpy.restore();
// //         });

// //         it('should handle CONNECTION_STATUS event', () => {
// //             const message = JSON.stringify({ eventType: EventType.CONNECTION_STATUS, data: { connected: true } });
// //             const logSpy = sinon.spy(console, 'log');
// //             service.handleMessage('ip', fakeSocket, { data: message }, 'mission1');
// //             expect(logSpy.calledWithMatch('[ROS Message Handler] 🌐 État de connexion')).to.be.true;
// //             logSpy.restore();
// //         });

//         it('should handle IDENTIFICATION event with started and completed status', () => {
//             const startedMsg = JSON.stringify({ eventType: EventType.IDENTIFICATION, data: { status: 'identification started' } });
//             service.handleMessage('ip', fakeSocket, { data: startedMsg }, 'mission1');
//             expect(socketEmitSpy.calledWith(ws_Methods.STATUS, 'Identification en cours')).to.be.true;
//             expect(socketEmitSpy.calledWith(ws_Methods.IDENTIFY, true)).to.be.true;

//             const completeMsg = JSON.stringify({ eventType: EventType.IDENTIFICATION, data: { status: 'completed' } });
//             service.handleMessage('ip', fakeSocket, { data: completeMsg }, 'mission1');
//             expect(socketEmitSpy.calledWith(ws_Methods.MISSION_AVAILABILITY, true)).to.be.true;
//             expect(socketEmitSpy.calledWith(ws_Methods.STATUS, 'Identification terminée')).to.be.true;
//             expect(socketEmitSpy.calledWith(ws_Methods.STOP_MISSION, false)).to.be.true;
//         });

// //         it('should handle invalid JSON', () => {
// //             const errorSpy = sinon.spy(console, 'error');
// //             service.handleMessage('ip', fakeSocket, { data: '{ invalidJson' }, 'mission1');
// //             expect(errorSpy.called).to.be.true;
// //             errorSpy.restore();
// //         });

// //         it('should handle unknown eventType gracefully', () => {
// //             const warnSpy = sinon.spy(console, 'warn');
// //             const msg = JSON.stringify({ eventType: 'UNKNOWN', data: {} });
// //             service.handleMessage('ip', fakeSocket, { data: msg }, 'mission1');
// //             expect(warnSpy.notCalled).to.be.true; // Because it's commented out in code
// //         });
// //     });

    
// // });
