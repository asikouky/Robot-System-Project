/* eslint-disable */
import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MenuBarComponent } from './menu-bar.component';
import { Router } from '@angular/router';

describe('MenuBarComponent', () => {
    let component: MenuBarComponent;
    let fixture: ComponentFixture<MenuBarComponent>;
    let mockRouter: jasmine.SpyObj<Router>;

    beforeEach(() => {
        mockRouter = jasmine.createSpyObj('Router', ['navigate'], { url: '/mission' });

        TestBed.configureTestingModule({
            imports: [MenuBarComponent],
            providers: [{ provide: Router, useValue: mockRouter }],
        });

        fixture = TestBed.createComponent(MenuBarComponent);
        component = fixture.componentInstance;
        fixture.detectChanges();
    });

    it('should create the component', () => {
        expect(component).toBeTruthy();
    });

    it('should toggle the menu and update state (open)', () => {
        spyOn<any>(component, 'updateMenuState');

        component.toggleMenu();

        expect(component.isMenuOpen).toBeTrue();
        expect((component as any).updateMenuState).toHaveBeenCalled();
    });

    it('should close the menu and update state', () => {
        component.isMenuOpen = true;
        spyOn<any>(component, 'updateMenuState');

        component.closeMenu();

        expect(component.isMenuOpen).toBeFalse();
        expect((component as any).updateMenuState).toHaveBeenCalled();
    });

    it('should navigate to route and close menu', () => {
        spyOn(console, 'log');
        spyOn(component, 'closeMenu');

        component.navigateTo('/home');

        expect(mockRouter.navigate).toHaveBeenCalledWith(['/home']);
        expect(console.log).toHaveBeenCalledWith('Navigating to:', '/home');
        expect(component.closeMenu).toHaveBeenCalled();
    });

    describe('updateMenuState', () => {
        let menuList: HTMLElement;
        let overlay: HTMLElement;
        let mainContainer: HTMLElement;

        beforeEach(() => {
            // Crée les éléments HTML simulés
            menuList = document.createElement('div');
            menuList.id = 'menuList';
            overlay = document.createElement('div');
            overlay.id = 'overlay';
            mainContainer = document.createElement('div');
            mainContainer.id = 'mainContainer';

            document.body.appendChild(menuList);
            document.body.appendChild(overlay);
            document.body.appendChild(mainContainer);
        });

        afterEach(() => {
            document.body.removeChild(menuList);
            document.body.removeChild(overlay);
            document.body.removeChild(mainContainer);
        });

        it('should add classes when menu is open', () => {
            component.isMenuOpen = true;
            (component as any).updateMenuState();

            expect(menuList.classList.contains('show')).toBeFalse();
            expect(overlay.classList.contains('active')).toBeFalse();
            expect(mainContainer.classList.contains('blurred')).toBeTrue();
            expect(mainContainer.classList.contains('no-interaction')).toBeTrue();
        });

        it('should remove classes when menu is closed', () => {
            menuList.classList.add('show');
            overlay.classList.add('active');
            mainContainer.classList.add('blurred', 'no-interaction');

            component.isMenuOpen = false;
            (component as any).updateMenuState();

            expect(menuList.classList.contains('show')).toBeTrue();
            expect(overlay.classList.contains('active')).toBeTrue();
            expect(mainContainer.classList.contains('blurred')).toBeFalse();
            expect(mainContainer.classList.contains('no-interaction')).toBeFalse();
        });

        it('should not add blur if current route is not /mission', () => {
            Object.defineProperty(mockRouter, 'url', { value: '/other' });
            component.isMenuOpen = true;
            (component as any).updateMenuState();

            expect(mainContainer.classList.contains('blurred')).toBeFalse();
        });
    });
});
