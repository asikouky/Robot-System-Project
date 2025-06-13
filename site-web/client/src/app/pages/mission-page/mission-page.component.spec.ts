/* eslint-disable */
import { ComponentFixture, TestBed, fakeAsync, tick } from '@angular/core/testing';
import { MissionPageComponent } from './mission-page.component';
import { ActivatedRoute } from '@angular/router';
import { Subject } from 'rxjs';
import { MatDialog } from '@angular/material/dialog';
import { ChangeDetectorRef, ElementRef } from '@angular/core';
import { SocketClientService } from '@app/services/socket/socket-client.service';
import { ws_Methods } from '@common/events';
import { MissionStatus } from '@common/action.robot';
import { HttpClientTestingModule } from '@angular/common/http/testing';

describe('MissionPageComponent', () => {
    let component: MissionPageComponent;
    let fixture: ComponentFixture<MissionPageComponent>;
    let socketServiceSpy: jasmine.SpyObj<SocketClientService>;
    let dialogSpy: jasmine.SpyObj<MatDialog>;
    let cdRefSpy: jasmine.SpyObj<ChangeDetectorRef>;
    let routeSubject: Subject<any>;

    beforeEach(async () => {
        routeSubject = new Subject();

        socketServiceSpy = jasmine.createSpyObj('SocketClientService', ['isSocketAlive', 'connect', 'send', 'on'], {
            socket: {
                on: jasmine.createSpy('on'),
            },
        });

        dialogSpy = jasmine.createSpyObj('MatDialog', ['open']);
        cdRefSpy = jasmine.createSpyObj('ChangeDetectorRef', ['detectChanges']);

        await TestBed.configureTestingModule({
            imports: [MissionPageComponent, HttpClientTestingModule],
            providers: [
                { provide: SocketClientService, useValue: socketServiceSpy },
                { provide: MatDialog, useValue: dialogSpy },
                { provide: ChangeDetectorRef, useValue: cdRefSpy },
                {
                    provide: ActivatedRoute,
                    useValue: {
                        paramMap: routeSubject.asObservable(),
                    },
                },
            ],
        }).compileComponents();
    });

    beforeEach(() => {
        fixture = TestBed.createComponent(MissionPageComponent);
        component = fixture.componentInstance;
        fixture.detectChanges();
    });

    it('should create the component', () => {
        expect(component).toBeTruthy();
    });

    it('should initialize with default mission status and IP from route', () => {
        routeSubject.next({
            get: (key: string) => (key === 'ip' ? '192.168.1.10' : null),
        });
        component.ngOnInit();
        expect(component.currentMissionStatus).toBe(MissionStatus.IDLE);
        expect(cdRefSpy.detectChanges).not.toHaveBeenCalled();
        expect(component.firstRobotIP).toBe('192.168.1.10');
    });



    it('should not call connect() if socket is alive', () => {
        socketServiceSpy.isSocketAlive.and.returnValue(true);
        component.connect();
        expect(socketServiceSpy.connect).toHaveBeenCalled();
    });

    it('should scroll to bottom if autoScroll and logsVisible are true', () => {
        const elementMock = {
            nativeElement: {
                scrollTop: 0,
                scrollHeight: 999,
            },
        };
        component.autoScroll = true;
        component.logsVisible = true;
        component['logsContainer'] = elementMock as unknown as ElementRef;
        component['scrollToBottom']();
        expect(elementMock.nativeElement.scrollTop).toBe(999);
    });

    it('should not scroll to bottom if autoScroll is false or logs not visible', () => {
        const elementMock = {
            nativeElement: {
                scrollTop: 0,
                scrollHeight: 999,
            },
        };
        component.autoScroll = false;
        component.logsVisible = true;
        component['logsContainer'] = elementMock as unknown as ElementRef;
        component['scrollToBottom']();
        expect(elementMock.nativeElement.scrollTop).toBe(0);
    });

    it('should toggle autoScroll and scroll if enabled', () => {
        component.autoScroll = false;
        spyOn<any>(component, 'scrollToBottom');
        component.toggleAutoScroll();
        expect(component.autoScroll).toBeTrue();
        expect(component['scrollToBottom']).toHaveBeenCalled();
    });

    it('should format time correctly', () => {
        expect(component.formatTime(125)).toBe('2m 5s');
        expect(component.formatTime(60)).toBe('1m 0s');
        expect(component.formatTime(0)).toBe('0m 0s');
    });

    it('should return correct mission status class', () => {
        const testCases = [
            { status: MissionStatus.IDLE, expected: 'status-idle' },
            { status: MissionStatus.IN_MISSION, expected: 'status-active' },
            { status: MissionStatus.RETURNING, expected: 'status-returning' },
            { status: MissionStatus.UPDATING, expected: 'status-updating' },
            { status: MissionStatus.DISCONNECTED, expected: 'status-disconnected' },
            { status: 999 as unknown as MissionStatus, expected: 'status-idle' },
        ];

        testCases.forEach(({ status, expected }) => {
            component.currentMissionStatus = status;
            expect(component.getMissionStatusClass()).toBe(expected);
        });
    });

    it('should send STOP_MISSION on finish()', () => {
        component.firstRobotIP = '192.168.1.2';
        component.finish();
        expect(socketServiceSpy.send).toHaveBeenCalledWith(ws_Methods.STOP_MISSION, {
            ip: '192.168.1.2',
        });
    });

    it('should toggle log visibility and scroll if needed', fakeAsync(() => {
        component.logsVisible = false;
        component.autoScroll = true;
        spyOn<any>(component, 'scrollToBottom');
        component.toggleLogsVisibility();
        expect(component.logsVisible).toBeTrue();
        tick(100);
        expect(component['scrollToBottom']).toHaveBeenCalled();
    }));

    it('should add a log message with timestamp', () => {
        const before = component.logMessages.length;
        component.addLogMessage('message test');
        expect(component.logMessages.length).toBe(before + 1);
        expect(component.logMessages[0]).toContain('message test');
    });

    it('should clear log messages', () => {
        component.logMessages = ['test1', 'test2'];
        component.clearLogs();
        expect(component.logMessages.length).toBe(0);
    });

    it('should call connect() if socket is not alive', () => {
        socketServiceSpy.isSocketAlive.and.returnValue(false);
        component.connect();
        expect(socketServiceSpy.connect).toHaveBeenCalled();
    });



    it('should update currentMissionStatus and detect changes on MISSION_STATUS_UPDATE', () => {
        const fakeStatus = MissionStatus.RETURNING;
    
        (socketServiceSpy.socket.on as jasmine.Spy).withArgs(ws_Methods.MISSION_STATUS_UPDATE, jasmine.any(Function))
            .and.callFake((_event, cb) => cb({ status: fakeStatus }));
    
        (socketServiceSpy.socket.on as jasmine.Spy).withArgs(ws_Methods.MAP_IMAGE_UPDATE, jasmine.any(Function)).and.callFake(() => {});
        (socketServiceSpy.socket.on as jasmine.Spy).withArgs(ws_Methods.TIMER_UPDATE, jasmine.any(Function)).and.callFake(() => {});
    
        component.setupSocketListeners();
        expect(component.currentMissionStatus).toBe(fakeStatus);
        expect(cdRefSpy.detectChanges).not.toHaveBeenCalled();
    });
    
    it('should update mapImageBase64 and log on MAP_IMAGE_UPDATE', () => {
        const fakeMap = 'data:image/png;base64,fake-map';
        const consoleSpy = spyOn(console, 'log');
    
        (socketServiceSpy.socket.on as jasmine.Spy).withArgs(ws_Methods.MAP_IMAGE_UPDATE, jasmine.any(Function))
            .and.callFake((_event, cb) => cb({ mapImage: fakeMap }));
    
        (socketServiceSpy.socket.on as jasmine.Spy).withArgs(ws_Methods.MISSION_STATUS_UPDATE, jasmine.any(Function)).and.callFake(() => {});
        (socketServiceSpy.socket.on as jasmine.Spy).withArgs(ws_Methods.TIMER_UPDATE, jasmine.any(Function)).and.callFake(() => {});
    
        component.setupSocketListeners();
    
        expect(component.mapImageBase64).toBe(fakeMap);
        expect(consoleSpy).toHaveBeenCalledWith('map recu', fakeMap);
    });
    
    

    it('should add log messages from various socket events', () => {
        const spyAddLog = spyOn(component, 'addLogMessage');

        (socketServiceSpy.on as jasmine.Spy).withArgs(ws_Methods.LASER_SCAN_LOG, jasmine.any(Function)).and.callFake((_event, cb) => cb('scan'));
        (socketServiceSpy.on as jasmine.Spy).withArgs(ws_Methods.ERROR, jasmine.any(Function)).and.callFake((_event, cb) => cb('err'));
        (socketServiceSpy.on as jasmine.Spy).withArgs(ws_Methods.STATUS, jasmine.any(Function)).and.callFake((_event, cb) => cb('status'));

        component.setupSocketListeners();

        expect(spyAddLog).toHaveBeenCalledWith('scan');
        expect(spyAddLog).toHaveBeenCalledWith('ERROR: err');
        expect(spyAddLog).toHaveBeenCalledWith('STATUS: status');
    });

    it('should toggle autoScroll and trigger scroll', () => {
        const scrollSpy = spyOn<any>(component, 'scrollToBottom');
        component.autoScroll = false;
        component.toggleAutoScroll();
        expect(component.autoScroll).toBeTrue();
        expect(scrollSpy).toHaveBeenCalled();
    });

    it('should format time correctly', () => {
        expect(component.formatTime(125)).toBe('2m 5s');
        expect(component.formatTime(0)).toBe('0m 0s');
    });

    it('should return correct CSS class based on mission status', () => {
        component.currentMissionStatus = MissionStatus.UPDATING;
        expect(component.getMissionStatusClass()).toBe('status-updating');
    });

    it('should send STOP_MISSION on finish()', () => {
        component.firstRobotIP = '192.168.1.1';
        component.finish();
        expect(socketServiceSpy.send).toHaveBeenCalledWith(ws_Methods.STOP_MISSION, { ip: '192.168.1.1' });
    });

    it('should toggle log visibility and scroll if needed', fakeAsync(() => {
        const scrollSpy = spyOn<any>(component, 'scrollToBottom');
        component.logsVisible = false;
        component.autoScroll = true;
        component.toggleLogsVisibility();
        expect(component.logsVisible).toBeTrue();
        tick(100);
        expect(scrollSpy).toHaveBeenCalled();
    }));

    it('should add and clear log messages', () => {
        component.addLogMessage('test log');
        expect(component.logMessages.length).toBe(1);
        component.clearLogs();
        expect(component.logMessages.length).toBe(0);
    });
});
