/* eslint-disable */
import { TestBed } from '@angular/core/testing';
import { HttpClientTestingModule, HttpTestingController } from '@angular/common/http/testing';
import { RobotService } from './robot.service';

describe('RobotService', () => {
    let service: RobotService;
    let httpMock: HttpTestingController;

    beforeEach(() => {
        TestBed.configureTestingModule({
            imports: [HttpClientTestingModule],
            providers: [RobotService],
        });
        service = TestBed.inject(RobotService);
        httpMock = TestBed.inject(HttpTestingController);
    });

    afterEach(() => {
        httpMock.verify();
    });

    it('should be created', () => {
        expect(service).toBeTruthy();
    });

    it('should call startRobot and make a POST request with the correct body', () => {
        const ip = '192.168.0.1';
        const mockResponse = { success: true };

        service.startRobot(ip).subscribe((response) => {
            expect(response).toEqual(mockResponse);
        });

        const req = httpMock.expectOne('http://localhost:3000/api/robot/start-robot');
        expect(req.request.method).toBe('POST');
        expect(req.request.body).toEqual({ ip });

        req.flush(mockResponse);
    });
});
