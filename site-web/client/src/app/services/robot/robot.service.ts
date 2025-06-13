import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';
@Injectable({
    providedIn: 'root',
})
export class RobotService {
    private apiUrl = 'http://localhost:3000/api/robot/start-robot'; // Backend Node.js

    constructor(private http: HttpClient) {}

    startRobot(ip: string): Observable<any> {
        return this.http.post(this.apiUrl, { ip });
    }
}
