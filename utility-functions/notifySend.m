
function notifySend( message, title, imagePath)
%NOTIFYSEND Send simple on screen display(OSD) notification to your linux
% desktop via notifyOSD and libnotify.
%
% Notify Send
% version 1.0.0.0 (12.9 KB) by Luc Trudeau
% https://www.mathworks.com/matlabcentral/fileexchange/28809-notify-send
% usages:
% 	notifySend( MESSAGE )
%	notifySend( MESSAGE, TITLE )
%	notifySend( MESSAGE, TITLE, IMAGEPATH )
%
% where,
%	MESSAGE is the message that will be displayed in the OSD. The message can
%		support HTML tags see http://ubuntuforums.org/showthread.php?t=1411620
%	TITLE is the title that will be displayed in bold over your message. If not
%		specified, the default value is "Matlab".
%	IMAGEPATH Specifies the image filename displayed in the OSD. If not
%		specified, the default value is the Matlab icon included in this
%		package.
%
% Dependencies:
%	The program 'notify-send' must be installed. On a ubuntu system, you can
%	install it by typing:
%		sudo apt-get install 
if nargin < 2
    title = 'Matlab';
end
if nargin < 3
    imagePath = which('matlabIcon.png');
end
[status result] = system(['notify-send -i "' imagePath '" "' title '" "' message '"']);
if status ~= 0
    warning('NotifySend:Error', [ 'Error executing notify-send, messages will not be shown. \n' result ]);
end
