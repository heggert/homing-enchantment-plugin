package art.chibi.homingenchant;

import org.bukkit.plugin.java.JavaPlugin;

public class HomingEnchantmentPlugin extends JavaPlugin {

    @Override
    public void onEnable() {
        getServer().getPluginManager().registerEvents(new HomingEnchantmentListener(), this);
    }
}