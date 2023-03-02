function w_data = unityWrite(client, data)
    if ~isstring(data)
        w_data = string(data);
    else
        w_data = data;
    end

    fopen(client);
    fwrite(client, w_data);
    fclose(client);
end