function u = crpa_doa(az_deg, el_deg)
%CRPA_DOA Unit vector towards a source at azimuth AZ (from +x to +y) and elevation EL [deg].
az = az_deg * pi / 180; el = el_deg * pi / 180;
u = [cos(el) .* cos(az); cos(el) .* sin(az); sin(el)];
end
