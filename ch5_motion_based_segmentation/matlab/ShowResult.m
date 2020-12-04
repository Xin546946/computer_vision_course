

function ShowResult( currentIm , backgroundMask , backgroundIm );

    figure( 100001 );
    subplot( 1 , 3 , 1 );       imshow( currentIm , [] );
    subplot( 1 , 3 , 2 );       imshow( 1 - backgroundMask , [] );
    subplot( 1 , 3 , 3 );       imshow( backgroundIm , [] );
    pause( 0.1 );

end