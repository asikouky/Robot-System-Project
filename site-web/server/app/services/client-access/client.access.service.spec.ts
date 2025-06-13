import { expect } from 'chai';
import { ClientAccessService } from '@app/services/client-access/client.access.service';

describe('ClientAccessService', () => {
    let service: ClientAccessService;

    beforeEach(() => {
        service = new ClientAccessService();
    });

    it('devrait enregistrer un client avec une IP et un socketId', () => {
        service.registerClient('192.168.0.1', 'socket123');

        expect(service.isIpRegistered('192.168.0.1')).to.be.true;
        expect(service.isClientConnected('192.168.0.1', 'socket123')).to.be.true;
    });

    it('devrait retourner false si l\'IP n\'est pas enregistrée', () => {
        expect(service.isIpRegistered('10.0.0.1')).to.be.false;
    });

    it('devrait retourner false si le socketId ne correspond pas', () => {
        service.registerClient('192.168.0.1', 'socket123');

        expect(service.isClientConnected('192.168.0.1', 'wrongSocket')).to.be.false;
    });

    it('devrait retirer un client avec son socketId et retourner l\'IP associée', () => {
        service.registerClient('192.168.0.1', 'socket123');
        const removedIp = service.removeClientBySocket('socket123');

        expect(removedIp).to.equal('192.168.0.1');
        expect(service.isIpRegistered('192.168.0.1')).to.be.false;
    });

    it('devrait retourner une chaîne vide si aucun client ne correspond au socketId', () => {
        service.registerClient('192.168.0.1', 'socket123');
        const removedIp = service.removeClientBySocket('socket999');

        expect(removedIp).to.equal('');
        expect(service.isIpRegistered('192.168.0.1')).to.be.true;
    });
});
