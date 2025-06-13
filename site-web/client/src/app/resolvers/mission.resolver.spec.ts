/* eslint-disable */
import { TestBed } from '@angular/core/testing';
import { MissionResolver } from './mission.resolver';
import { Router } from '@angular/router';

describe('MissionResolver', () => {
    let resolver: MissionResolver;
    let routerSpy: jasmine.SpyObj<Router>;

    beforeEach(() => {
        const routerSpyObj = jasmine.createSpyObj('Router', ['navigate']);
        TestBed.configureTestingModule({
            providers: [MissionResolver, { provide: Router, useValue: routerSpyObj }],
        });
        resolver = TestBed.inject(MissionResolver);
        routerSpy = TestBed.inject(Router) as jasmine.SpyObj<Router>;
    });

    it('should be created', () => {
        expect(resolver).toBeTruthy();
    });

    describe('#resolve', () => {
        it('should be a function', () => {
            expect(typeof resolver.resolve).toBe('function');
        });

        it('should return an Observable', () => {
            const result = resolver.resolve();
            expect(result).toBeDefined();
            expect(result.subscribe).toBeDefined();
        });

        it('should navigate to /home and return false if page is reloaded', (done) => {
            
            const originalNavigation = performance.navigation;

            
            Object.defineProperty(performance, 'navigation', {
                value: { type: 1, TYPE_RELOAD: 1 },
                configurable: true,
            });

            spyOn(console, 'log');

            const result$ = resolver.resolve();
            result$.subscribe((value) => {
               
                expect(console.log).toHaveBeenCalledWith('[Resolver] Rafraîchissement détecté, redirection vers /home');
                
                expect(routerSpy.navigate).toHaveBeenCalledWith(['/home']);
                
                expect(value).toBeFalse();

                
                Object.defineProperty(performance, 'navigation', { value: originalNavigation });
                done();
            });
        });

        it('should return true if page is not reloaded', (done) => {
            
            const originalNavigation = performance.navigation;

            
            Object.defineProperty(performance, 'navigation', {
                value: { type: 0, TYPE_RELOAD: 1 },
                configurable: true,
            });

            spyOn(console, 'log');

            const result$ = resolver.resolve();
            result$.subscribe((value) => {
                
                expect(console.log).toHaveBeenCalledWith('[Resolver] Accès autorisé');
               
                expect(routerSpy.navigate).not.toHaveBeenCalled();
                
                expect(value).toBeTrue();

                
                Object.defineProperty(performance, 'navigation', { value: originalNavigation });
                done();
            });
        });
    });
});
