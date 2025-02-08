function E = errore_3link(Q)
    global P
    Rn = CinematicaDiretta(Q(1), Q(2), Q(3)); % matrice di cinematica diretta
    PT = [Rn(1,4); Rn(2,4); Rn(3,4)];
    E = 0.5*(norm(P-PT)^2);
end