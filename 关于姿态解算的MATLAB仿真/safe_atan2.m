function [ result ] = safe_atan2( x,y )
    
	if isnan(x/y)
		if x>0
		  result=  pi/2.0; 
        else
            if x<0
                result= -pi/2.0;
            else 
                result= 0.0;
            end
        end
    else
        result= atan2(x,y);
    end
end

