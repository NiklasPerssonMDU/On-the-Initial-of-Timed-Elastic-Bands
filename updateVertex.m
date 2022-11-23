function [parents ,g, openQueue,f]=updateVertex (s, sPrim, parents, g, h, f, openQueue, map)


%get old cost for sPrim
gOld = g(sPrim(1),sPrim(2));


%%
%Get parent of node s
sParent=cell2mat(parents(s(1),s(2)));



if LOS2(sParent, sPrim, map)  %if the s parents node is in line of sight of sprim, do
    %Compute the new g value using the g value of the parents + the
    %cost to go to the parent of s from sPrim
    
    gNew = g(sParent(1),sParent(2)) + euclidanDistance([sParent(1),sParent(2)], sPrim);
    if gNew < gOld
        g(sPrim(1), sPrim(2)) = gNew;
        parents{sPrim(1), sPrim(2)} = sParent;
        if openQueue(sPrim(1), sPrim(2)) 
            openQueue(sPrim(1), sPrim(2)) = false;
        end
        openQueue(sPrim(1), sPrim(2)) = true;
        f(sPrim(1), sPrim(2)) = g(sPrim(1),sPrim(2)) + h(sPrim(1),sPrim(2));
    end

else % Sprim can not see s parent but can see s
    gNew = g(s(1), s(2)) + euclidanDistance([s(1), s(2)], sPrim);
    if gNew < gOld
        g(sPrim(1), sPrim(2)) = gNew;
        parents{sPrim(1), sPrim(2)} = s;

        if openQueue(sPrim(1), sPrim(2))
            openQueue(sPrim(1), sPrim(2)) = false;
        end


        openQueue(sPrim(1), sPrim(2)) = true;
        f(sPrim(1), sPrim(2)) = g(sPrim(1),sPrim(2)) + h(sPrim(1),sPrim(2));
    end


end



end