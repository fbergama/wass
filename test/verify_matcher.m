function status = verify_matcher( input_frames, CONFIG_DIR )
%VERIFY_MATCHER 

status = 'ok';
MIN_NUMBER_OF_MATCHES=400;
MAX_EPI_ERROR=0.5;
T_MAX_ERR = 2E-2;
R_MAX_ERR = 5E-3;

Rgt = load_opencv_xml_matrix( [CONFIG_DIR,'/ext_R.xml'] );
Tgt = load_opencv_xml_matrix( [CONFIG_DIR,'/ext_T.xml'] );

for ii=1:numel(input_frames)
    wdir = input_frames{ii}.wd;
    
    aux = dlmread( [wdir,'/matcher_stats.csv'],';',1,0);
    nmatches = floor( aux(1) );
    if nmatches  < MIN_NUMBER_OF_MATCHES
        status = sprintf('%s: Too few matches. (%d<%d)',wdir, nmatches, MIN_NUMBER_OF_MATCHES);
        return;
    end
    
    avgepierror = aux(2);
    if avgepierror  > MAX_EPI_ERROR
        status = sprintf('%s: Avg. epi error too high. (%f>%f)',wdir, avgepierror, MAX_EPI_ERROR);
        return;
    end
    
    % Load RT
    R = load_opencv_xml_matrix( [wdir,'/ext_R.xml'] );
    T = load_opencv_xml_matrix( [wdir,'/ext_T.xml'] );
    
    tmaxerr = max( abs(T-Tgt ) );
    rmaxerr = max( max( abs(R-Rgt ) ) );
    
    if tmaxerr  > T_MAX_ERR
        status = sprintf('%s: tmaxerr too high. (%f>%f)',wdir, tmaxerr, T_MAX_ERR);
        return;
    end
    if rmaxerr  > R_MAX_ERR
        status = sprintf('%s: rmaxerr too high. (%f>%f)',wdir, rmaxerr, R_MAX_ERR);
        return;
    end
end

end

