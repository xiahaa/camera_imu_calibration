function h = getfigure(name,sz,position)
    h = struct();
    h.fig = figure('Name',name, 'NumberTitle','off', 'Menubar','none', ...
        'Pointer','cross', 'Resize','off', 'Position',[200 200 sz(2) sz(1)]);
    if ~mexopencv.isOctave()
        %HACK: not implemented in Octave
        movegui(h.fig, position);
    end
    h.ax = axes('Parent',h.fig, 'Units','normalized', 'Position',[0 0 1 1]);
end