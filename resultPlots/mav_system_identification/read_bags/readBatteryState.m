function battery = readBatteryState(bag, topic)

battery_raw = bag.readAll(topic);

sz = length(battery_raw)


battery.t = zeros(1, sz);
battery.i = zeros(1, sz);
battery.p = zeros(1, sz);
battery.v = zeros(1, sz);


for i=1:sz
   battery.t(i) = timestampFromHeader(battery_raw{i}.header);
   battery.i(i) = battery_raw{i}.header.seq;
   battery.p(:,i) = battery_raw{i}.percentage;
   battery.v(:,i) = battery_raw{i}.voltage;
end