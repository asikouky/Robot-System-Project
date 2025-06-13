import { Injectable } from '@angular/core';
import { Resolve, Router } from '@angular/router';
import { Observable, of } from 'rxjs';

@Injectable({
    providedIn: 'root',
})
export class MissionResolver implements Resolve<boolean> {
    constructor(private router: Router) {}

    resolve(): Observable<boolean> {
        console.log('[Resolver] Vérification du rafraîchissement');

    
        if (performance.navigation.type === performance.navigation.TYPE_RELOAD) {
            console.log('[Resolver] Rafraîchissement détecté, redirection vers /home');
            this.router.navigate(['/home']); 
            return of(false); 
        }

        console.log('[Resolver] Accès autorisé');
        return of(true); 
    }
}
