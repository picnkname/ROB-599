function xSat = sat(x, lim)
    % Saturate x to be between +/- lim
    xSat = min(max(x,-lim),lim);
end