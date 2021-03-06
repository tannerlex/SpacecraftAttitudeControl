function [wdotstar_P, wstar_P, qstar_PI] = ...
    command_input(wmax, wdotmax, q_PB, qs_BI, ql_BI, t, switchtimes)

wstar_P = [0;0;0];
wdotstar_P = [0;0;0];
if t < switchtimes(1)
    % sun pointing
    qstar_PI = qX(q_PB, qs_BI); % convert to principle frame

elseif t < switchtimes(2)
    % slew between sun pointing and ground tracking
    qs_PI = qX(q_PB, qs_BI); % convert to principle frame
    ql_PI = qX(q_PB, ql_BI); % convert to principle frame
    qs_IP = qUnit(A2q(q2A(qs_PI)'));
    qe = qUnit(qX(ql_PI, qs_IP)); % error quaternion
    [ehat, th_f] = q2e(qe);
    th_0 = 0;
    [th, w, wdot] = ...
        profiled_scalar(th_0, th_f, t-switchtimes(1), wdotmax, wmax);
    qr = qUnit(e2q(ehat, th));
    qstar_PI = qUnit(qX(qr, qs_PI));
    wstar_P = q2A(q_PB) * (ehat * w);
    wdotstar_P = q2A(q_PB) * (ehat * wdot);

elseif t < switchtimes(3)
    % track ground target
    qstar_PI = qX(q_PB, ql_BI);

elseif t < switchtimes(4)
    % slew between ground tracking and sun pointing
    qs_PI = qX(q_PB, qs_BI); % convert to principle frame
    ql_PI = qX(q_PB, ql_BI); % convert to principle frame
    ql_IP = qUnit(A2q(q2A(ql_PI)'));
    qe = qUnit(qX(qs_PI, ql_IP)); % error quaternion
    [ehat, th_f] = q2e(qe);
    th_0 = 0;
    [th, w, wdot] = ...
        profiled_scalar(th_0, th_f, t-switchtimes(3), wdotmax, wmax);
    qr = qUnit(e2q(ehat, th));
    qstar_PI = qUnit(qX(qr, ql_PI));
    wstar_P = q2A(q_PB) * (ehat * w);
    wdotstar_P = q2A(q_PB) * (ehat * wdot);

else % t >= switchtimes(4)
    % sun pointing
    qstar_PI = qX(q_PB, qs_BI);
end
