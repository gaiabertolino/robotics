function ris = cont(x,y)
    if (x > 4 & x < 9 & y > 0 & y < 1)
    ris = true;
    elseif (x > 10 & x < 14 & y > 0.5 & y < 2.5)
    ris =  true;
    elseif (x > 3.5 & x < 6.5 & y > 3 & y < 5)
    ris = true;
    elseif (x > 8 & x < 9 & y > 4 & y < 5)
    ris = true;
    elseif (x > 11 & x < 13 & y > 4 & y < 8)
    ris = true;
    elseif (x > 2 & x < 7 & y > 7 & y < 10)
    ris = true;
    else ris = false;
    end
end