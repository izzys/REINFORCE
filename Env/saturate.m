function uOut = saturate(uIn,limit)
    uOut=uIn;
    if uIn>limit; uOut=limit; end
    if uIn<-limit; uOut=-limit; end
end

