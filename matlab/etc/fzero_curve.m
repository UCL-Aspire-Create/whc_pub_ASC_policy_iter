function xi = fzero_curve(x,f,p,doPlot)
%#codegen
% 
% Multiple usage of this matlab function:
% I.      xi = fzero_curve(x,f,c,doPlot)
% Find the intersection points between a line segment of slope 1 and a curve: find
% all xi such that xi = f(xi) + c, where the function f(.) is NOT known
% explicitly but, instead, only some points on it. The pair of points (x,f)
% define the given curve. The results of this code represent the best
% approximations one can get in terms of (linear) interpolation. Please
% take note that only solutions xi within the x vector's range are seeked;
% outside this range, or if no solution is found at all, the output xi =
% NaN is issued. Use doPlot ~= 0 in order to plot some meningful figures
% with results.
% 
% Inputs: 
% x      1 x n    vector defining the x-axis points of the curve (x,f(x))
% f      1 x n    vector defining the y-axis points of the curve (x,f(x))
% c      1 x 1    scalar value
% doPlot 1 x 1    scalar value, optional usage 
% 
% Outputs:
% xi     1 x m   vector of all x-axis coordinates where the best
% approximations of xi = f(xi) + c are obtained by interpolation;   
% 
% Examples: the curve is defined as triangle wave signal around the line 
% Ex1: x=0:1:6; f=x-sawtooth( (2*pi/4)*x,0.5); c=0;    xi=fzero_curve(x,f,c,1) 
% Ex2: x=0:1:6; f=x+sawtooth( (2*pi/4)*x,0.5); c=0;    xi=fzero_curve(x,f,c,1) 
% Ex3: x=0:1:6; f=x+sawtooth( (2*pi/4)*x,0.5); c=-0.4; xi=fzero_curve(x,f,c,1)
%
% Example: x and f are identiqually equal
% Ex4: x=0:1:3; f=x; c=0; xi=fzero_curve(x,f,c,1)
%
% An alternative providing the same output is the matlab function
% intersections(x,x,x,f+c).
%
%
% II.      xi = fzero_curve(x,f,[b c],doPlot) 
% Find the intersection points between an enclined line segment of slope b
% and a curve: find all xi such that g(xi)=f(xi), where the linear function
% g(xi)=b*xi-c; the function f(.) is NOT known explicitly but, instead,
% only some points on it. The pair of points (x,f) define the given curve.
% The results of this code represent the best approximations one can get in
% terms of (linear) interpolation. Please take note that only solutions xi
% within the x vector's range are seeked; outside this range, or if no
% solution is found at all, the output xi = NaN is issued. Use doPlot ~= 0
% in order to plot some meningful figures with results.
% 
% Inputs: 
% x       1 x n    vector defining the x-axis points of the curve (x,f(x))
% f       1 x n    vector defining the y-axis points of the curve (x,f(x))
% [b c]   2 x 1    b and c are scalar values used to build the linear 
%                  function g(x)=b*x+c 
% doPlot  1 x 1    scalar value, optional usage 
% 
% Outputs:
% xi     1 x m   vector of all x-axis coordinates where the best
% approximations of g(xi) = f(xi) are obtained by interpolation; 
%
% Examples:
%  x=0:1:6; f=x-sawtooth( (2*pi/4)*x,0.5); b=1.2; c=0; xi=fzero_curve(x,f,[b c],1)
%  c=0; fzero_curve([0:5 6:11],[2.1:-1:-3.1 2.1+10:-1:-3.1+10],c, 1)
%  
% % code by ctdr 2014-07


%% set default values to the optional matlab input variables 
if nargin < 4 %if ~exist('doPlot','var')  %i.e. if nargin <= 3 
    doPlot = 0;
end


%% partI. check/ensure consistency of the input data
if ~isequal(length(x),length(f)) 
    warning('!! Vectors x and f should have the same length !!');
    xi = nan;
    return
end

% make sure both x and f are line vectors
if ~isrow(x) 
    x = x';
end

if ~isrow(f) 
    f = f';
end

%% partII. check input data and consequently take decisions 

% pre-process1: Treat points (x,f) that are nan: Option1) remove them from within vectors; Option2) duplicate previous/next point and them make sure they will not be accounted by fzero_.. algo 

%{-
%var1: (apparently) codegen-ready 
idx = find(~isnan(x));
x = x(idx); %overwrite 
f = f(idx); %overwrite 

idx = find(~isnan(f)); %identify all indices of elements that are NOT nan
x = x(idx); %overwrite 
f = f(idx); %overwrite 
%}

%{
%var2: codegen-ready
n = 0; %ini: this variable acts as index along x and f, respectively 
while n<length(x) %up to this line of code we are sure that length(x)==length(f)   
    n=n+1;
    if isnan(x(n))  %check if current x is nan
        if n == 1
            x(n) = x(n+1); %zoh_fwd
            f(n) = f(n+1); %zoh_fwd
        else %i.e. n>=2
            x(n) = x(n-1); %zoh_bwd
            f(n) = f(n-1); %zoh_bwd
        end %if
    end %if
    
    if isnan(f(n)) %check if current f is nan
        if n == 1
            x(n) = x(n+1); %zoh_fwd
            f(n) = f(n+1);
        else %i.e. n>=2
            x(n) = x(n-1); %zoh_bwd
            f(n) = f(n-1);
        end %if
    end %if            
        
end %while
%}

%pre-process2: remove all duplicate points (x,f), e.g. x=[1 2 2 3 4]; f=[2 3 4 5 6], by keeping the 1st apparition only   
[x_new,ix_old,~] = unique(x); %unique apparitions of x 
f_new = f(ix_old); %associated f values
 x = x_new; %overwrite 
 f = f_new; %overwrite 

% %preprocess3: if we end up with no points (x,f), it does not make sense to calculate intersection any more 
if isequal(min([size(x) size(f)]),0)
    xi = nan;
    return
end

 
%% interpret matlab input variables 
%extract [b c] from p
if isequal(length(p),1)
    b = 1;
    c = p;
elseif isequal(length(p),2) 
    b = p(1);
    c = p(2);
else
    error('!! Please check varargin{3} !!');
    return
end

%% Start algorithm

% initialization
xiaux = nan(1,max(length(x),1)); 
nr = 1;

y = b*x-f-c;
idxs = find(mcv(diff( y >= 0 )));   %mcv(.) is only for codegen purpose    
%%V&V: figure; plot(x,y,'b-',x(1:end),y>=0,'r*'); grid on; xlabel('x-axis'); ylabel('y(x)=x-f(x)-c');       

%Now, idxs contains the indices of points situated precisely on the x-axis
%together with points that are situated right before a crossing of x-axis by the curve y(x)=x-f-c.
%Hereafter we want to separate them: the former be used directly to
%construct xi, while the latter be used for interpolation between
%each of these points and the adjacent ones. 
i = find(y==0);
if ~isempty(i)
    xiaux(1:length(i)) = x(i);
    idxs = setdiff(idxs,[i-1 i]); %then we already know that no crossing of x-axis takes place between xi-1<->xi and xi<->xi+1, i.e. on these branches no intersection points can be found so we'll exclude them from the set idxs used for interp1(.) 
    nr = length(i)+1;
end

%% the interpolation should be done based on points where we know a crossing of x-axis has taken place between them and the adjacent ones.  
%{
%var1 [NOT codegen-ready]
for jj=idxs
    xiaux(nr) = interp1([y(jj) y(jj+1)],[x(jj) x(jj+1)],0,'linear',nan); % find the point xi situated on x-axis where y intersects a horizontal line passing through zero on abscissa
    nr = nr + 1;
end
%}

%var2 [codegen-ready] equivalent to hereabove code
if ~isempty(idxs)
    j=0;
    while j<length(idxs)
        j = j+1; %idx inside vector idxs 
        xiaux(nr) = interp1([y(idxs(j)) y(idxs(j)+1)],[x(idxs(j)) x(idxs(j)+1)],0,'linear',nan); % find the point xi situated on x-axis where y intersects a horizontal line passing through zero on abscissa
        nr = nr+1;
    end %while
end %if

%% define xi
xi = xiaux(1:max(1,nr-1)); %xi should return at least one nan 
xi = unique(xi); %sort in ascending order + ensure they appear uniquely; this uniqueness ensurance might be redundant here     


%% plot results 
if doPlot ~= 0
    precision = '%.3f';
    % use these figures for verification and validation of results
    figure('Position',[10 60 600 800]); 
    
    ha = nan(4,1); %ini: codegen purpose
    ha(1) = subplot(411); 
    plot(x,y,'b.-',  [x(1) x(end)],0*[1 1],'k:',  xi,0,'go',    'linewidth',3); 
    grid on; axis tight;
    xlabel('x-axis');    
    if length(xi) == 1
        legend('curve (x,y(x)=b*x-f(x)-c)','x-axis',['inters. point (xi=',num2str(xi,precision),',0)'],'location','South');
    else
        legend('curve (x,y(x)=b*x-f(x)-c)','x-axis',['inters. points (xi=[',num2str(xi,horzcat(precision,'; ')),'],0)'],'location','South');
    end
    
    title(horzcat('Identification of intersection points:\newline{}','xi are the x-axis coordinates of these intersection points\newline{}',['b=',num2str(b,precision),'; c=',num2str(c,precision),'; ' ]));
    
    ha(2) = subplot(412);
    plot(x,b*x-f,'b.-',  [x(1) x(end)],c*[1 1],'k:',  xi,c,'go',     'linewidth',3); 
    grid on; axis tight;
    xlabel('x-axis');
    ylabel('Intersection of\newline{}b*x-f(x) with c');
    if length(xi) == 1
        legend('curve (x,b*x-f(x))','horiz line of ordinate c',['inters. point (xi=',num2str(xi,precision),',c)'],'location','South');
    else
        legend('curve (x,b*x-f(x))','horiz line of ordinate c',['inters. points xi=([',num2str(xi,horzcat(precision,'; ')),'],c)'],'location','South');
    end
    
    ha(3) = subplot(413);    
    plot(x,b*x,'b.-',   x,f+c,'r.-',   reshape(ones(3,1)*xi, 1,[]), repmat([min(min(b*x),min(f+c)) max(max(b*x),max(f+c)) min(min(b*x),min(f+c))], 1,length(xi) ), 'k+:',    'linewidth',3); 
    %                                ^ =[xi(1) xi(1) xi(1)  xi(2) xi(2) xi(2) ...]; este doar un artificiu pt afisare corecta a liniei intrerupte; poate ca sunt si alte solutii mai bune!  repmat(..)=[val_min val_max val_min ...]                  
    grid on; axis tight;
    xlabel('x-axis');
    ylabel('Intersection of\newline{}b*x with f(x)+c');
    if length(xi) == 1
        legend('line segm. (x,b*x)','curve (x,f(x)+c)',['inters. point xi=',num2str(xi,precision)],'location','South');
    else
        legend('line segm. (x,b*x)','curve (x,f(x)+c)',['inters points xi=[',num2str(xi,horzcat(precision,'; ')),']'],'location','South')
    end
    
    ha(4) = subplot(414);
    plot(x,b*x-c,'b-',      x,f,'r-',      reshape(ones(3,1)*xi, 1,[]), repmat([min(min(b*x-c),min(f)) max(max(b*x-c),max(f)) min(min(b*x-c),min(f))], 1,length(xi) ), 'k+:',    'linewidth',3);
    grid on; axis tight; 
    xlabel('x-axis');
    ylabel('Intersection of\newline{}g(x)=b*x-c with f(x)'); 
    if length(xi) == 1 
        legend('g(x)=b*x-c','f(x)',['inters. point xi=',num2str(xi,precision)],'location','South');
    else 
        legend('g(x)=b*x-c','f(x)',['inters points xi=[',num2str(xi,horzcat(precision,'; ')),']'],'location','South')
    end 
    
    linkaxes(ha,'x'); %zoom/pan simultaneously on the x-axis    
end


%% alternative code based on numerical optimization: use fzero(.), but this is more time-consuming;  


