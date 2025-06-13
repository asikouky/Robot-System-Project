import { MissionRoomService } from '@app/services/socket/roomServices/mission-room/mission.room.service';
import * as http from 'http';
import * as io from 'socket.io';

export class SocketManager {
    missionRoomService: MissionRoomService;

    private sio: io.Server;

    constructor(server: http.Server) {
        this.sio = new io.Server(server, { cors: { origin: '*', methods: ['GET', 'POST'] } });
        this.missionRoomService = new MissionRoomService(this.sio);
    }

    handleSockets(): void {
        this.sio.on('connection', (socket) => {
            console.log(`Connexion par l'utilisateur avec id : ${socket.id}`); // eslint-disable-line no-console

            socket.emit('codeFileContent', 'Current code content here...'); // Send the current code content to the client

            this.missionRoomService.handleSockets(socket);

            socket.on('disconnect', (reason) => {
                console.log(`Deconnexion par l'utilisateur avec id : ${socket.id}`); // eslint-disable-line no-console
                console.log(`Raison de deconnexion : ${reason}`); // eslint-disable-line no-console
            });
        });
    }
}
