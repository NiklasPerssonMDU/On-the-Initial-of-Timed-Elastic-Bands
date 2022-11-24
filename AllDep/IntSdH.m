function [IAdH, ISdH] = IntSdH(heading)
        Ts=0.1;
        %we compute both the Integral Squared heading derivative and...
        ISdH = sum( ( atan2(sin(diff(heading)), cos(diff(heading)) )  ./Ts).^2 );

        %the Integral of Absolute heading derivative
        IAdH = sum( abs(( atan2(sin(diff(heading)), cos(diff(heading)) )./Ts)) );
end