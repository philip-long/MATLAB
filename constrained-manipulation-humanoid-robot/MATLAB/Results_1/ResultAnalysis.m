clear all,clc,close all

ResultFiles=dir('*.mat');

%% find the failures

wpstar_sqp_fail=0;
wpstar_fail=0;
wp_sqp_fail=0;
wp_fail=0;
for i=1:length(ResultFiles)
    if(contains(ResultFiles(i).name,'Failsqp_wpstar'))
        wpstar_sqp_fail=wpstar_sqp_fail+1;
    elseif(contains(ResultFiles(i).name,'Failwpstar'))
        wpstar_fail=wpstar_fail+1;
    elseif(contains(ResultFiles(i).name,'Failwp'))
        wp_fail=wp_fail+1;
    elseif(contains(ResultFiles(i).name,'Failsqp_wp'))    
        wp_sqp_fail=wp_sqp_fail+1;
    end
end
y=[wpstar_fail,wp_fail,30
    wpstar_sqp_fail,wp_sqp_fail,22
    ];
c = categorical({'interior-point','sqp'});
bar(c,y);
legend('wp* failures','wp failures','total runs')

figure(4)
c = categorical({'CMP','MP'});
y=[wpstar_sqp_fail+wpstar_fail,52
    wp_sqp_fail+wp_fail,52];
bar(c,y);
legend('Failures','Total Runs')



%% Numerical Analysis
%
%  Conclusion: In general if the wpstar succeeds, the resulting
%  manipulability is generally better and has slightly more success
%  in IK queries.
%
%
bar_exitflags=[];
wp_results=[];
wpstar_results=[];
SEPARATE=false;
SQP=true; % ANALYSE SQP RESULTS TRUE==SQP FALSE=INTERIOR_POINT
for j=1:20
    wp_result_valid=false;
    wpstar_result_valid=false;
    for i=1:length(ResultFiles)
        if(contains(ResultFiles(i).name,'Fail'))
            continue;
        elseif(SEPARATE && SQP~=contains(ResultFiles(i).name,'sqp')) %     
            continue;
        elseif(contains(ResultFiles(i).name,strcat('wp_t',int2str(j),'o')))
            WP_RESULTS=load(ResultFiles(i).name);    
            wp_result_valid=true;
        elseif(contains(ResultFiles(i).name,strcat('wpstar_t',int2str(j),'o')))
            WPSTAR_RESULTS=load(ResultFiles(i).name);            
            wpstar_result_valid=true;
        end        
    end
    if(wp_result_valid && wpstar_result_valid)
        bar_exitflags=[bar_exitflags; ...
                       length(find(WP_RESULTS.EXITFLAG==1)) length(find(WPSTAR_RESULTS.EXITFLAG==1))];   
        wp_results=[wp_results;... 
                        mean(WP_RESULTS.COST(find(WP_RESULTS.EXITFLAG==1))) mean(WPSTAR_RESULTS.COST(find(WPSTAR_RESULTS.EXITFLAG==1)))];
        wpstar_results=[wpstar_results;...
                        mean(WP_RESULTS.WPSTAR(find(WP_RESULTS.EXITFLAG==1))) mean(WPSTAR_RESULTS.WPSTAR(find(WPSTAR_RESULTS.EXITFLAG==1)))];        
    end    
end
%%
figure(2)
%bar(bar_exitflags);
subplot(1,3,1)
c=categorical({'CMP','MP'});
bar(c,[mean(bar_exitflags(:,2)),mean(bar_exitflags(:,1))],'FaceColor',[.8 .8 .8],'EdgeColor',[0 0. 0.],'LineWidth',1.5)
hold on
errorbar([mean(bar_exitflags(:,2));mean(bar_exitflags(:,1))],[std(bar_exitflags(:,2));std(bar_exitflags(:,1))],'k.')
title('IK Successes /8','interpreter','latex')

%figure(2)
%bar(wpstar_results)
%legend('wp ','wpstar')

subplot(1,3,2)
%c=categorical({'MP','CMP'});
bar(c,[mean(wpstar_results(:,2));mean(wpstar_results(:,1))],'FaceColor',[.8 .8 .8],'EdgeColor',[0 0. 0.],'LineWidth',1.5)
hold on
errorbar([mean(wpstar_results(:,2));mean(wpstar_results(:,1))],[std(wpstar_results(:,2));std(wpstar_results(:,1))],'k.')
title('$w_p^{*}$','interpreter','latex')

%figure(3)
%bar(wp_results)
%legend('wp ','wpstar')
%title('Manipulability')
subplot(1,3,3)
%c=categorical({'MP','CMP'});
bar(c,[mean(wp_results(:,2));mean(wp_results(:,1))],'FaceColor',[.8 .8 .8],'EdgeColor',[0 0. 0.],'LineWidth',1.5)
hold on
errorbar([mean(wp_results(:,2));mean(wp_results(:,1))],[std(wp_results(:,2));std(wp_results(:,1))],'k.')
title('$w_p$','interpreter','latex')


