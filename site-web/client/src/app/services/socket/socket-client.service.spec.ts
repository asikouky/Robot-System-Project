import { TestBed } from '@angular/core/testing';
import { SocketClientService } from './socket-client.service';
import { Socket } from 'socket.io-client';


describe('SocketClientService', () => {
    let service: SocketClientService;
    let mockSocket: any;

    beforeEach(() => {
        spyOn(console, 'log');
        spyOn(console, 'error');

        mockSocket = {
            connected: false,
            id: 'test-id',
            on: jasmine.createSpy('on'),
            emit: jasmine.createSpy('emit'),
            disconnect: jasmine.createSpy('disconnect'),
        };

        spyOn<any>(require('socket.io-client'), 'io').and.returnValue(mockSocket);

        TestBed.configureTestingModule({});
        service = TestBed.inject(SocketClientService);
    });

    it('should be created', () => {
        expect(service).toBeTruthy();
    });


    it('should return true when socket is connected', () => {
        service.socket = { connected: true } as Socket;
        expect(service.isSocketAlive()).toBeTrue();
    });

    it('should return false when socket is not connected', () => {
        service.socket = { connected: false } as Socket;
        expect(service.isSocketAlive()).toBeFalse();
    });

    it('should call disconnect on socket', () => {
        service.socket = mockSocket;
        service.disconnect();
        expect(mockSocket.disconnect).toHaveBeenCalled();
    });

    it('should register an event listener with "on"', () => {
        service.socket = mockSocket;
        const callback = jasmine.createSpy('callback');
        service.on('custom-event', callback);
        expect(mockSocket.on).toHaveBeenCalledWith('custom-event', callback);
    });

    it('should emit an event without callback', () => {
        service.socket = mockSocket;
        service.send('event', { foo: 'bar' });
        expect(mockSocket.emit).toHaveBeenCalledWith('event', { foo: 'bar' });
    });

    it('should emit an event with callback', () => {
        service.socket = mockSocket;
        const cb = jasmine.createSpy('cb');
        service.send('event', { foo: 'bar' }, cb);
        expect(mockSocket.emit).toHaveBeenCalledWith('event', { foo: 'bar' }, cb);
    });

    it('should emit without data if only callback is passed', () => {
        service.socket = mockSocket;
        const cb = jasmine.createSpy('cb');
        service.send('event', undefined, cb);
        expect(mockSocket.emit).toHaveBeenCalledWith('event', cb);
    });

    
    
    
});
