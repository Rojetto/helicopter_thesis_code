function run_engine()
    engine_name = 'matlab_heli';
    if ~matlab.engine.isEngineShared()
        matlab.engine.shareEngine(engine_name)
        disp(['MATLAB engine running, connect to ''' engine_name ''''])
    else
        disp(['Already running under ''' matlab.engine.engineName ''''])
    end
end

