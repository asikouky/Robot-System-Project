import { Service } from 'typedi';

@Service()
export class ClientAccessService {
    private activeClients: Map<string, string> = new Map(); // Map <IP, socketId>

    registerClient(ip: string, socketId: string): void {
        this.activeClients.set(ip, socketId);
        console.log(`[ClientAccessService] L'IP ${ip} a été attribuée au client ${socketId}.`);
    }

    removeClientBySocket(socketId: string): string {
        for (const [ip, id] of this.activeClients.entries()) {
            if (id === socketId) {
                this.activeClients.delete(ip);
                console.log(`[ClientAccessService] IP ${ip} libérée (Client ${socketId} déconnecté).`);
                return ip;
            }
        }
        return '';
    }

    isIpRegistered(ip: string): boolean {
        return this.activeClients.has(ip);
    }

    isClientConnected(ip: string, socketId: string): boolean {
        return this.activeClients.get(ip) === socketId;
    }
}
