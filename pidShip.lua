



---------------------------Utils------------------------------------
function RotateVectorByQuat(quat, v)
    local x = quat.x * 2
    local y = quat.y * 2
    local z = quat.z * 2
    local xx = quat.x * x
    local yy = quat.y * y
    local zz = quat.z * z
    local xy = quat.x * y
    local xz = quat.x * z
    local yz = quat.y * z
    local wx = quat.w * x
    local wy = quat.w * y
    local wz = quat.w * z
    local res = {}
    res.x = (1.0 - (yy + zz)) * v.x + (xy - wz) * v.y + (xz + wy) * v.z
    res.y = (xy + wz) * v.x + (1.0 - (xx + zz)) * v.y + (yz - wx) * v.z
    res.z = (xz - wy) * v.x + (yz + wx) * v.y + (1.0 - (xx + yy)) * v.z
    return res
end
function RotateVectorByConjugateQuat(q, v)
    local conj = { w = q.w, x = -q.x, y = -q.y, z = -q.z }
    return RotateVectorByQuat(conj, v)
end
local preQuat = ship.getQuaternion()
local function getEularRate(q)
    local quat = q
    local omega = {}
    local pX = RotateVectorByQuat(quat, { x = 1, y = 0, z = 0 })
    local pY = RotateVectorByQuat(quat, { x = 0, y = 1, z = 0 })
    local pZ = RotateVectorByQuat(quat, { x = 0, y = 0, z = -1 })

    local XPoint = { x = pX.x, y = pX.y, z = pX.z }
    local ZPoint = { x = pZ.x, y = pZ.y, z = pZ.z }
    preQuat.x = -preQuat.x
    preQuat.y = -preQuat.y
    preQuat.z = -preQuat.z
    XPoint = RotateVectorByQuat(preQuat, XPoint)
    ZPoint = RotateVectorByQuat(preQuat, ZPoint)
    omega.pitch = math.deg(math.asin(ZPoint.y))
    omega.roll = math.deg(math.asin(XPoint.y))
    omega.yaw = math.deg(math.atan2(-XPoint.z, XPoint.x))
    preQuat = quat
    return {
        r = omega.roll,
        y = omega.yaw,
        p = omega.pitch
    }
end
function PIDController(kp, ki, kd, setpoint, measurement, prev_error, integral,max)
    local error = setpoint - measurement

    local proportional = kp * error

    integral = integral + error
    local integral_term = ki * integral

    local derivative = kd * (error - prev_error)

    local output = proportional + integral_term + derivative

    if output > max then
        output = max
    elseif output < -max then
        output = -max
    end

    return output, error, integral
end
---------------------------Velocity---------------------------------
local kp = 28 --38
local ki = 2  --2
local kd = 28 --30
local max = 9000000;

local prev_errorx = 0.0
local integralx = 0.0
local outputx, new_errorx, new_integralx  = 0,0,0


local prev_errory = 0.0
local integraly = 0.0
local outputy, new_errory, new_integraly  = 0,0,0

local prev_errorz = 0.0
local integralz = 0.0
local outputz, new_errorz, new_integralz  = 0,0,0
-------------------------------Omega---------------------------------
local kp2 = 10
local ki2 = 0 
local kd2 = 1
local max2 = 100000;

local prev_errorr = 0.0
local integralr = 0.0
local outputr, new_errorr, new_integralr  = 0,0,0

local prev_errorp = 0.0
local integralp = 0.0
local outputp, new_errorp, new_integralp  = 0,0,0

local prev_errora= 0.0
local integrala = 0.0
local outputa, new_errora, new_integrala  = 0,0,0

-------------------------------Api-----------------------------------
function setShipRotVelo(x,y,z)
    local q = ship.getQuaternion()
    local v = ship.getVelocity()
    v = RotateVectorByConjugateQuat(q,v)
    local M = ship.getMass()

    outputx, new_errorx, new_integralx = PIDController(kp, ki, kd, x, v.x, prev_errorx, integralx,max)
    outputy, new_errory, new_integraly = PIDController(kp, ki, kd, y, v.y, prev_errory, integraly,max)
    outputz, new_errorz, new_integralz = PIDController(kp, ki, kd, z, v.z, prev_errorz, integralz,max)

    prev_errorx = new_errorx
    integralx = new_integralx
    prev_errory = new_errory
    integraly = new_integraly
    prev_errorz = new_errorz
    integralz = new_integralz

    ship.applyRotDependentForce(outputx*M,outputy*M,outputz*M)
end

function setShipWorldVelo(x,y,z)
    local v = ship.getVelocity()
    local M = ship.getMass()

    outputx, new_errorx, new_integralx = PIDController(kp, ki, kd, x, v.x, prev_errorx, integralx,max)
    outputy, new_errory, new_integraly = PIDController(kp, ki, kd, y, v.y, prev_errory, integraly,max)
    outputz, new_errorz, new_integralz = PIDController(kp, ki, kd, z, v.z, prev_errorz, integralz,max)

    prev_errorx = new_errorx
    integralx = new_integralx
    prev_errory = new_errory
    integraly = new_integraly
    prev_errorz = new_errorz
    integralz = new_integralz

    ship.applyInvariantForce(outputx*M,outputy*M,outputz*M)
end

function setShipOmega(r,p,y)
    local q = ship.getQuaternion()
    local eulerAngles = getEularRate(q)
    local I = ship.getMomentOfInertiaTensor()[1][1]

    outputr, new_errorr, new_integralr = PIDController(kp2, ki2, kd2,r, eulerAngles.r, prev_errorr, integralr,max2)
    outputp, new_errorp, new_integralp = PIDController(kp2, ki2, kd2,p, eulerAngles.p, prev_errorp, integralp,max2)
    outputa, new_errora, new_integrala = PIDController(kp2, ki2, kd2,y, eulerAngles.y, prev_errora, integrala,max2)

    prev_errorr = new_errorr
    integralr = new_integralr
    prev_errorp = new_errorp
    integralp = new_integralp
    prev_errora = new_errora
    integrala = new_integrala

    ship.applyRotDependentTorque(outputp*I,outputa*I,outputr*I)
end
--------------------------Example-----------------------------------
local x_speed  = 0
local y_speed  = 0
local z_speed  = 0
local r_angle = 0
local p_angle = 0
local a_angle = 0

local normal = 20
local a_normal = 8
local kk=1
local go = false

--local l = peripheral.wrap("back")
--l.setTextScale(0.5)
--term.redirect(l)
local P = peripheral.wrap("top")
rs.setOutput("back",false)

while true do
    if P.hasUser() then
        --l.clear()
        --l.setCursorPos(1,1)
        
        if P.getButton(4) then
            y_speed = normal
        elseif P.getButton(1) then y_speed  = -normal
        else y_speed  = 0
        end
        if P.getButton(12) then
            go = true
        elseif P.getButton(14) then
            go = false
        end
        if go then
            z_speed = normal
        else
            z_speed = 0
        end
        if P.getButton(3)  then
            x_speed  = normal
        elseif P.getButton(2)  then
            x_speed  = -normal
        else x_speed  =0
        end
        
        if P.getButton(5)  then
          kk = 5
        else kk=1
        end

        if P.getButton(15) then
            a_angle = a_normal;
        elseif P.getButton(13) then
            a_angle = -a_normal;
        else a_angle  =0
        end
        if (P.getAxis(2)==-1)then
            p_angle = a_normal;
        elseif (P.getAxis(2)==1) then
            p_angle = -a_normal;
        else p_angle  =0
        end
        if (P.getAxis(1)==1) then
            r_angle = a_normal;
        elseif (P.getAxis(1)==-1)  then
            r_angle = -a_normal;
        else r_angle  = 0
        end
        if (P.getButton(11)) then
            rs.setOutput("back",true)
        else rs.setOutput("back",false)
        end

        setShipOmega(r_angle,p_angle,a_angle)
        setShipRotVelo(x_speed,y_speed,z_speed*kk)
    end
    sleep(0.05)
end
--------------------------End--------------------------------------